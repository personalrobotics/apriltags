/***********************************************************************

Copyright (c) 2014, Carnegie Mellon University
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*************************************************************************/
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/Marker.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

// #include <src/TagDetector.h>
// #include <src/TagDetection.h>
// #include <src/TagFamily.h>

// #include "apriltagscpp/TagDetector.h"
// #include "apriltagscpp/TagDetection.h"
// #include "apriltagscpp/TagFamily.h"

#include "TagDetector.h"
#include "TagDetection.h"
#include "TagFamily.h"


// #include <TagDetector.h>
// #include <TagDetection.h>
// #include <TagFamily.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <visualization_msgs/MarkerArray.h>
#include "yaml-cpp/yaml.h"
#include <sstream>
#include <fstream>

#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/make_shared.hpp>

#include "apriltags.h"
#include <apriltags/AprilTagDetections.h>


using namespace std;

// Functions

double GetTagSize(int tag_id)
{
    boost::unordered_map<size_t, double>::iterator tag_sizes_it =
        tag_sizes_.find(tag_id);
    if(tag_sizes_it != tag_sizes_.end()) {
        return tag_sizes_it->second;
    } else {
        return default_tag_size_;
    }
}

void GetMarkerTransformUsingOpenCV(const TagDetection& detection, Eigen::Matrix4d& transform, cv::Mat& rvec, cv::Mat& tvec)
{
    // Check if fx,fy or cx,cy are not set
    if ((camera_info_.K[0] == 0.0) || (camera_info_.K[4] == 0.0) || (camera_info_.K[2] == 0.0) || (camera_info_.K[5] == 0.0))
    {
        ROS_WARN("Warning: Camera intrinsic matrix K is not set, can't recover 3D pose");
    }

    double tag_size = GetTagSize(detection.id);

    std::vector<cv::Point3f> object_pts;
    std::vector<cv::Point2f> image_pts;
    double tag_radius = tag_size/2.;
    
    object_pts.push_back(cv::Point3f(-tag_radius, -tag_radius, 0));
    object_pts.push_back(cv::Point3f( tag_radius, -tag_radius, 0));
    object_pts.push_back(cv::Point3f( tag_radius,  tag_radius, 0));
    object_pts.push_back(cv::Point3f(-tag_radius,  tag_radius, 0));
    
    image_pts.push_back(detection.p[0]);
    image_pts.push_back(detection.p[1]);
    image_pts.push_back(detection.p[2]);
    image_pts.push_back(detection.p[3]);

    cv::Matx33f intrinsics(camera_info_.K[0], 0, camera_info_.K[2],
                           0, camera_info_.K[4], camera_info_.K[5],
                           0, 0, 1);
    
    cv::Vec4f distortion_coeff(camera_info_.D[0], camera_info_.D[1], camera_info_.D[2], camera_info_.D[3]);

    // Estimate 3D pose of tag
    // Methods:
    //   CV_ITERATIVE
    //     Iterative method based on Levenberg-Marquardt optimization.
    //     Finds the pose that minimizes reprojection error, being the sum of squared distances
    //     between the observed projections (image_points) and the projected points (object_pts).
    //   CV_P3P
    //     Based on: Gao et al, "Complete Solution Classification for the Perspective-Three-Point Problem"
    //     Requires exactly four object and image points.
    //   CV_EPNP
    //     Moreno-Noguer, Lepetit & Fua, "EPnP: Efficient Perspective-n-Point Camera Pose Estimation"
    int method = CV_ITERATIVE;
    bool use_extrinsic_guess = false; // only used for ITERATIVE method
    cv::solvePnP(object_pts, image_pts, intrinsics, distortion_coeff, rvec, tvec, use_extrinsic_guess, method);

    cv::Matx33d r;
    cv::Rodrigues(rvec, r);
    Eigen::Matrix3d rot;
    rot << r(0,0), r(0,1), r(0,2),
           r(1,0), r(1,1), r(1,2),
           r(2,0), r(2,1), r(2,2);

    Eigen::Matrix4d T;
    T.topLeftCorner(3,3) = rot;
    T.col(3).head(3) <<
            tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
    T.row(3) << 0,0,0,1;
    
    transform = T;
}

// Draw a line with an arrow head
// This function's argument list is designed to match the cv::line() function
void ArrowLine(cv::Mat& image,
               const cv::Point& pt1, const cv::Point& pt2, const cv::Scalar& color,
               const int thickness=1, const int line_type=8, const int shift=0,
               const double tip_length=0.1)
{
    // Normalize the size of the tip depending on the length of the arrow
    const double tip_size = norm(pt1-pt2)*tip_length;

    cv::line(image, pt1, pt2, color, thickness, line_type, shift);
    const double angle = atan2(double(pt1.y - pt2.y), double(pt1.x - pt2.x));
    cv::Point p(cvRound(pt2.x + tip_size * cos(angle + CV_PI / 4.0)),
    cvRound(pt2.y + tip_size * sin(angle + CV_PI / 4.0)));
    cv::line(image, p, pt2, color, thickness, line_type, shift);
    p.x = cvRound(pt2.x + tip_size * cos(angle - CV_PI / 4.0));
    p.y = cvRound(pt2.y + tip_size * sin(angle - CV_PI / 4.0));
    cv::line(image, p, pt2, color, thickness, line_type, shift);
}

// Draw the marker's axes with red/green/blue lines
void DrawMarkerAxes(const cv::Matx33f& intrinsic_matrix, const cv::Vec4f& distortion_coeffs,
                    const cv::Mat& rvec, const cv::Mat& tvec, const float length, const bool use_arrows,
                    cv::Mat& image)
{
    std::vector<cv::Point3f> axis_points;
    axis_points.push_back(cv::Point3f(0, 0, 0));
    axis_points.push_back(cv::Point3f(length, 0, 0));
    axis_points.push_back(cv::Point3f(0, length, 0));
    axis_points.push_back(cv::Point3f(0, 0, length));
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(axis_points, rvec, tvec, intrinsic_matrix, distortion_coeffs, image_points);

    // Draw axis lines
    const int thickness = 2;
    if (use_arrows)
    {
        ArrowLine(image, image_points[0], image_points[1], cv::Scalar(0, 0, 255), thickness);
        ArrowLine(image, image_points[0], image_points[2], cv::Scalar(0, 255, 0), thickness);
        ArrowLine(image, image_points[0], image_points[3], cv::Scalar(255, 0, 0), thickness);
    }
    else
    {
        cv::line(image, image_points[0], image_points[1], cv::Scalar(0, 0, 255), thickness);
        cv::line(image, image_points[0], image_points[2], cv::Scalar(0, 255, 0), thickness);
        cv::line(image, image_points[0], image_points[3], cv::Scalar(255, 0, 0), thickness);
    }
}

// Draw the outline of the square marker in a single color,
// with a mark on the first corner
void DrawMarkerOutline(const TagDetection& detection, const cv::Scalar outline_color, cv::Mat& image)
{
    // Draw outline
    const int outline_thickness = 2;
    for(int i = 0; i < 4; i++) {
        cv::Point2f p0(detection.p[i].x, detection.p[i].y);
        cv::Point2f p1(detection.p[(i + 1) % 4].x, detection.p[(i + 1) % 4].y);
        cv::line(image, p0, p1, outline_color, outline_thickness);
    }
    // Indicate first corner with a small rectangle
    const int width = 6;
    const int rect_thickness = 1;
    cv::Point2f p0(detection.p[0].x - width/2, detection.p[0].y - width/2);
    cv::Point2f p1(detection.p[0].x + width/2, detection.p[0].y + width/2);
    cv::rectangle(image, p0, p1, outline_color, rect_thickness, CV_AA); // anti-aliased
}

// Draw the four edges of the marker in separate colors
// to show the ordering of corner points
void DrawMarkerEdges(const TagDetection& detection, cv::Mat& image)
{
    // Draw edges
    std::vector<cv::Scalar> colors;

    colors.push_back(cv::Scalar(0,0,255)); // red (BGR ordering)
    colors.push_back(cv::Scalar(0,255,0)); // green
    colors.push_back(cv::Scalar(255,0,0)); // blue
    colors.push_back(cv::Scalar(0,204,255)); // yellow

    const int edge_thickness = 2;
    for(int i = 0; i < 4; i++) {
        cv::Point2f p0(detection.p[i].x, detection.p[i].y);
        cv::Point2f p1(detection.p[(i + 1) % 4].x, detection.p[(i + 1) % 4].y);
        cv::line(image, p0, p1, colors[i], edge_thickness);
    }
}

// Draw the marker's ID number
void DrawMarkerID(const TagDetection& detection, const cv::Scalar text_color, cv::Mat& image)
{
    cv::Point2f center(0,0);
    for(int i = 0; i < 4; i++)
    {
        center += cv::Point2f(detection.p[i].x, detection.p[i].y);
    }
    center.x = center.x / 4.0;
    center.y = center.y / 4.0;
    std::stringstream s;
    s << "  " << detection.id; // move label away from origin
    const double font_scale = 0.5;
    const int thickness = 2;
    cv::putText(image, s.str(), center, cv::FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
}

// Callback for camera info
void InfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info)
{
    camera_info_ = (*camera_info);
    has_camera_info_ = true;
}

// Callback for image data
void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if(!has_camera_info_){
        ROS_WARN("No Camera Info Received Yet");
        return;
    }

    // Get the image
    cv_bridge::CvImagePtr subscribed_ptr;
    try
    {
        subscribed_ptr = cv_bridge::toCvCopy(msg, "mono8");
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    cv::Mat subscribed_gray = subscribed_ptr->image;
    cv::Point2d opticalCenter;

    if ((camera_info_.K[2] > 1.0) && (camera_info_.K[5] > 1.0))
    {
        // cx,cy from intrinsic matric look reasonable, so we'll use that
        opticalCenter = cv::Point2d(camera_info_.K[5], camera_info_.K[2]);
    }
    else
    {
        opticalCenter = cv::Point2d(0.5*subscribed_gray.rows, 0.5*subscribed_gray.cols);
    }

    // Detect AprilTag markers in the image
    TagDetectionArray detections;
    detector_->process(subscribed_gray, opticalCenter, detections);
    visualization_msgs::MarkerArray marker_transforms;
    apriltags::AprilTagDetections apriltag_detections;
    apriltag_detections.header.frame_id = msg->header.frame_id;
    apriltag_detections.header.stamp = msg->header.stamp;

    cv_bridge::CvImagePtr subscribed_color_ptr;
    if ((viewer_) || (publish_detections_image_))
    {
        try
        {
            subscribed_color_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        if (display_marker_overlay_)
        {
            // Overlay a black&white marker for each detection
            subscribed_color_ptr->image = family_->superimposeDetections(subscribed_color_ptr->image, detections);
        }
    }

    for(unsigned int i = 0; i < detections.size(); ++i)
    {
        // skip bad detections
        if(!detections[i].good)
        {
            continue;
        }

        Eigen::Matrix4d pose;
        cv::Mat rvec;
        cv::Mat tvec;
        GetMarkerTransformUsingOpenCV(detections[i], pose, rvec, tvec);
        
        // Get this info from earlier code, don't extract it again
        Eigen::Matrix3d R = pose.block<3,3>(0,0);
        Eigen::Quaternion<double> q(R);
        
        double tag_size = GetTagSize(detections[i].id);
        cout << tag_size << " " << detections[i].id << endl;
        
        // Fill in MarkerArray msg
        visualization_msgs::Marker marker_transform;
        marker_transform.header.frame_id = msg->header.frame_id;
        marker_transform.header.stamp = msg->header.stamp;

        // Only publish marker for 0.5 seconds after it
        // was last seen
        marker_transform.lifetime = ros::Duration(0.5);

        stringstream convert;
        convert << "tag" << detections[i].id;
        marker_transform.ns = convert.str().c_str();
        marker_transform.id = detections[i].id;
        if(display_type_ == "ARROW"){
            marker_transform.type = visualization_msgs::Marker::ARROW;
            marker_transform.scale.x = tag_size; // arrow length
            marker_transform.scale.y = tag_size/10.0; // diameter
            marker_transform.scale.z = tag_size/10.0; // diameter
        }
        else if(display_type_ == "CUBE"){
            marker_transform.type = visualization_msgs::Marker::CUBE;
            marker_transform.scale.x = tag_size;
            marker_transform.scale.y = tag_size;
            marker_transform.scale.z = marker_thickness_;
        }
        marker_transform.action = visualization_msgs::Marker::ADD;
        marker_transform.pose.position.x = pose(0,3);
        marker_transform.pose.position.y = pose(1,3);
        marker_transform.pose.position.z = pose(2,3);
        marker_transform.pose.orientation.x = q.x();
        marker_transform.pose.orientation.y = q.y();
        marker_transform.pose.orientation.z = q.z();
        marker_transform.pose.orientation.w = q.w();
        
        marker_transform.color.r = 1.0;
        marker_transform.color.g = 0.0;
        marker_transform.color.b = 1.0;
        marker_transform.color.a = 1.0;
        marker_transforms.markers.push_back(marker_transform);
        
        // Fill in AprilTag detection.
        apriltags::AprilTagDetection apriltag_det;
        apriltag_det.header = marker_transform.header;
        apriltag_det.id = marker_transform.id;
        apriltag_det.tag_size = tag_size;
        apriltag_det.pose = marker_transform.pose;
        const TagDetection &det = detections[i];
        for(uint pt_i = 0; pt_i < 4; ++pt_i)
        {
            geometry_msgs::Point32 img_pt;
            img_pt.x = det.p[pt_i].x;
            img_pt.y = det.p[pt_i].y;
            img_pt.z = 1;
            apriltag_det.corners2d[pt_i] = img_pt;
        }
        apriltag_detections.detections.push_back(apriltag_det);

        if ((viewer_) || (publish_detections_image_))
        {
            if (display_marker_outline_)
            {
                cv::Scalar outline_color(0, 0, 255); // blue (BGR ordering)
                DrawMarkerOutline(detections[i], outline_color, subscribed_color_ptr->image);
            }

            if (display_marker_id_)
            {
                cv::Scalar text_color(255, 255, 0); // light-blue (BGR ordering)
                DrawMarkerID(detections[i], text_color, subscribed_color_ptr->image);
            }

            if (display_marker_edges_)
            {
                DrawMarkerEdges(detections[i], subscribed_color_ptr->image);
            }

            if (display_marker_axes_)
            {
                cv::Matx33f intrinsics(camera_info_.K[0], 0, camera_info_.K[2],
                                       0, camera_info_.K[4], camera_info_.K[5],
                                       0, 0, 1);
                cv::Vec4f distortion_coeff(camera_info_.D[0], camera_info_.D[1],
                                           camera_info_.D[2], camera_info_.D[3]);
                double axis_length = tag_size;
                const bool draw_arrow_heads = true;
                DrawMarkerAxes(intrinsics, distortion_coeff, rvec, tvec, axis_length,
                               draw_arrow_heads, subscribed_color_ptr->image);
            }
        }
    }
    marker_publisher_.publish(marker_transforms);
    apriltag_publisher_.publish(apriltag_detections);

    if(publish_detections_image_)
    {
        image_publisher_.publish(subscribed_color_ptr->toImageMsg());
    }

    if(viewer_)
    {
        cv::imshow("AprilTags", subscribed_color_ptr->image);
    }
}

void ConnectCallback(const ros::SingleSubscriberPublisher& info)
{
    // Check for subscribers.
    uint32_t subscribers = marker_publisher_.getNumSubscribers()
                           + apriltag_publisher_.getNumSubscribers();
    ROS_DEBUG("Subscription detected! (%d subscribers)", subscribers);

    if(subscribers && !running_)
    {
        ROS_DEBUG("New Subscribers, Connecting to Input Image Topic.");
        ros::TransportHints ros_transport_hints(ros::TransportHints().tcpNoDelay());
        image_transport::TransportHints image_transport_hint(image_transport::TransportHints(
                                "raw", ros_transport_hints, (*node_),
                                "image_transport"));

        image_subscriber = (*image_).subscribe(
                DEFAULT_IMAGE_TOPIC, 1, &ImageCallback,
                image_transport_hint);
        info_subscriber = (*node_).subscribe(
                DEFAULT_CAMERA_INFO_TOPIC, 10, &InfoCallback);
        running_ = true;
    }
}

void DisconnectHandler()
{
}

void DisconnectCallback(const ros::SingleSubscriberPublisher& info)
{
    // Check for subscribers.
    uint32_t subscribers = marker_publisher_.getNumSubscribers()
                           + apriltag_publisher_.getNumSubscribers();
    ROS_DEBUG("Unsubscription detected! (%d subscribers)", subscribers);
    
    if(!subscribers && running_)
    {
        ROS_DEBUG("No Subscribers, Disconnecting from Input Image Topic.");
        image_subscriber.shutdown();
        info_subscriber.shutdown();
        running_ = false;
    }
}

void GetParameterValues()
{
    // Load node-wide configuration values.
    node_->param("tag_family", tag_family_name_, DEFAULT_TAG_FAMILY);
    node_->param("default_tag_size", default_tag_size_, DEFAULT_TAG_SIZE);
    node_->param("display_type", display_type_, DEFAULT_DISPLAY_TYPE);
    node_->param("marker_thickness", marker_thickness_, 0.01);

    node_->param("viewer", viewer_, false);
    node_->param("publish_detections_image", publish_detections_image_, false);
    node_->param("display_marker_overlay", display_marker_overlay_, true);
    node_->param("display_marker_outline", display_marker_outline_, false);
    node_->param("display_marker_id", display_marker_id_, false);
    node_->param("display_marker_edges", display_marker_edges_, false);
    node_->param("display_marker_axes", display_marker_axes_, false);

    ROS_INFO("Tag Family: %s", tag_family_name_.c_str());

    // Load tag specific configuration values.
    XmlRpc::XmlRpcValue tag_data;
    node_->param("tag_data", tag_data, tag_data);

    // Iterate through each tag in the configuration.
    XmlRpc::XmlRpcValue::ValueStruct::iterator it;
    for (it = tag_data.begin(); it != tag_data.end(); ++it)
    {
        // Retrieve the settings for the next tag.
        int tag_id = boost::lexical_cast<int>(it->first);
        XmlRpc::XmlRpcValue tag_values = it->second;

        // Load all the settings for this tag.
        if (tag_values.hasMember("size")) 
        {
            tag_sizes_[tag_id] = static_cast<double>(tag_values["size"]);
            ROS_DEBUG("Setting tag%d to size %f m.", tag_id, tag_sizes_[tag_id]);
        }
    }
}

void SetupPublisher()
{    
    ros::SubscriberStatusCallback connect_callback = &ConnectCallback;
    ros::SubscriberStatusCallback disconnect_callback = &DisconnectCallback;
    
    // Publisher
    marker_publisher_ = node_->advertise<visualization_msgs::MarkerArray>(
            DEFAULT_MARKER_TOPIC, 1, connect_callback,
            disconnect_callback);
    apriltag_publisher_ = node_->advertise<apriltags::AprilTagDetections>(
            DEFAULT_DETECTIONS_TOPIC, 1, connect_callback, disconnect_callback);

    if(publish_detections_image_)
    {
        image_publisher_ = (*image_).advertise(DEFAULT_DETECTIONS_IMAGE_TOPIC, 1);
    }
}

void InitializeTags()
{
    tag_params.newQuadAlgorithm = 1;
    family_ = new TagFamily(tag_family_name_);
    detector_ = new TagDetector(*family_, tag_params);
}

void InitializeROSNode(int argc, char **argv)
{
    ros::init(argc, argv, "apriltags");
    node_ =  boost::make_shared<ros::NodeHandle>("~");
    image_ = boost::make_shared<image_transport::ImageTransport>(*node_);
}

int main(int argc, char **argv)
{
    InitializeROSNode(argc,argv);
    GetParameterValues();
    SetupPublisher();
    InitializeTags();

    if(viewer_){
        cvNamedWindow("AprilTags");
        cvStartWindowThread();
    }

    ROS_INFO("AprilTags node started.");
    running_ = false;
    has_camera_info_ = false;
    ros::spin();
    ROS_INFO("AprilTags node stopped.");

    //Destroying Stuff
    cvDestroyWindow("AprilTags");
    delete detector_;
    delete family_;

    return EXIT_SUCCESS;
}
