#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/Marker.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include <src/TagDetector.h>
#include <src/TagDetection.h>
#include <src/TagFamily.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <visualization_msgs/MarkerArray.h>
#include "yaml-cpp/yaml.h"
#include <sstream>
#include <fstream>

#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/make_shared.hpp>

#include <apriltags/Start.h>
#include <apriltags/Stop.h>
#include <apriltags/StopAll.h>
#include <apriltags/IsRunning.h>
#include <apriltags/IsIdOpen.h>
#include <apriltags/RunningIds.h>
#include "apriltags.h"

using namespace std;

// Functions

Eigen::Matrix4d GetDetectionTransform(TagDetection detection){
    double tag_size = default_tag_size_;
    boost::unordered_map<size_t, double>::iterator tag_size_it =
            tag_sizes_.find(detection.id);
    if(tag_size_it != tag_sizes_.end()){
        tag_size = (*tag_size_it).second; 
    }
    
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
    
    cv::Mat rvec, tvec;
    cv::Vec4f dist_param(0,0,0,0);
    cv::solvePnP(object_pts, image_pts, intrinsics, dist_param,
            rvec, tvec);
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
    
    return T;
}

// Callback for camera info
void InfoCallback(
        const sensor_msgs::CameraInfoConstPtr& camera_info){
    if(running_){
        camera_info_ = (*camera_info);
    }
}

// Callback for image data
void ImageCallback(
        const sensor_msgs::ImageConstPtr& msg )
{
    // Only continue if the node is running
    if(!running_){
        return;
    }
    
    // Get the image
    cv_bridge::CvImagePtr subscribed_ptr;
    try{
        subscribed_ptr = cv_bridge::toCvCopy(msg, "mono8");
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    cv::Mat subscribed_gray = subscribed_ptr->image;
    cv::Mat tmp;
    cv::Point2d opticalCenter(0.5*subscribed_gray.rows,
                              0.5*subscribed_gray.cols);
    TagDetectionArray detections;
    detector_->process(subscribed_gray, opticalCenter, detections);
    visualization_msgs::MarkerArray marker_transforms;
    
    if(viewer_){
        subscribed_gray = family_->superimposeDetections(subscribed_gray,
                                                          detections);
    }
    for(unsigned int i = 0; i < detections.size(); ++i){
        
        Eigen::Matrix4d pose = GetDetectionTransform(detections[i]);
        
        // Get this info from earlier code, don't extract it again
        Eigen::Matrix3d R = pose.block<3,3>(0,0);
        Eigen::Quaternion<double> q(R);
        
        visualization_msgs::Marker marker_transform;
        marker_transform.header.frame_id = frame_;
        marker_transform.header.stamp = ros::Time::now();
        stringstream convert;
        convert << "tag" << detections[i].id;
        marker_transform.ns = convert.str().c_str();
        marker_transform.id = detections[i].id;
        marker_transform.type = visualization_msgs::Marker::CUBE;
        marker_transform.action = visualization_msgs::Marker::ADD;
        marker_transform.pose.position.x = pose(0,3);
        marker_transform.pose.position.y = pose(1,3);
        marker_transform.pose.position.z = pose(2,3);
        marker_transform.pose.orientation.x = q.x();
        marker_transform.pose.orientation.y = q.y();
        marker_transform.pose.orientation.z = q.z();
        marker_transform.pose.orientation.w = q.w();
        
        marker_transform.scale.x = 1.;//tag_size;
        marker_transform.scale.y = 1.;//tag_size;
        marker_transform.scale.z = 0.05;// * tag_size;
        marker_transform.color.r = 0.5;
        marker_transform.color.g = 0.5;
        marker_transform.color.b = 0.5;
        marker_transform.color.a = 1.0;
        marker_transforms.markers.push_back(marker_transform);
    }
    marker_publisher_.publish(marker_transforms);
    
    if(viewer_){
        cv::imshow("AprilTags", subscribed_gray);
    }
}

void ConnectCallback(const ros::SingleSubscriberPublisher& info){
    // Subscribers
    uint32_t subscribers = marker_publisher_.getNumSubscribers();
    printf("New Subscription Detected to AprilTags!! Now Have %d subscribers\n",subscribers);

    ros::TransportHints ros_transport_hints(ros::TransportHints().tcpNoDelay());
    image_transport::TransportHints image_transport_hint(image_transport::TransportHints(
                            "raw", ros_transport_hints, (*node_),
                            "image_transport"));

    image_subscriber = (*image_).subscribe(
            camera_topic_name, 1, &ImageCallback,
            image_transport_hint);
    info_subscriber = (*node_).subscribe(
            "/camera_info", 10, &InfoCallback);
    running_ = true;
}

void DisconnectHandler()
{
    running_ = false;
    image_subscriber.shutdown();
    info_subscriber.shutdown();
}

void DisconnectCallback(const ros::SingleSubscriberPublisher& info){
    uint32_t subscribers = marker_publisher_.getNumSubscribers();
    printf("Unsubscription Detected from AprilTags!! Now Have %d subscribers\n",subscribers);
    
    if(!subscribers)
    {
        printf("No Subscribers, Disconnecting from Input Image Topic.\n");
        DisconnectHandler();
    }
}

void GetParameterValues(){
    (*node_).param("viewer", viewer_, 0);
    (*node_).param("tag_family", tag_family_name_, DEFAULT_TAG_FAMILY);
    (*node_).param("tag_data", tag_data, string(""));
    (*node_).param("default_tag_size", default_tag_size_, SMALL_TAG_SIZE);
    (*node_).param("tf_frame", frame_, string("/prosilica_cam"));
}

void SetupPublisher(){
    
    ros::SubscriberStatusCallback connect_callback = &ConnectCallback;
    ros::SubscriberStatusCallback disconnect_callback = &DisconnectCallback;
    
    // Publisher
    marker_publisher_ = (*node_).advertise<visualization_msgs::MarkerArray>(
            output_marker_list_topic_name, 1, connect_callback,
            disconnect_callback);
}

void InitializeTags()
{
    tag_params.newQuadAlgorithm = 1;
    family_ = new TagFamily(tag_family_name_);
    detector_ = new TagDetector(*family_, tag_params);
}

void InitializeServices()
{
    running_ = false;
    stop_all_service_ = (*node_).advertiseService(
            "stop_all", &StopAllService);
    is_running_service_ = (*node_).advertiseService(
            "is_running", &IsRunningService);
}
    
// Stop All Service
bool StopAllService(apriltags::StopAll::Request &req,
                    apriltags::StopAll::Response &res){
    DisconnectHandler();
    return true;
}

// Is Running Service
bool IsRunningService(apriltags::IsRunning::Request &req,
                      apriltags::IsRunning::Response &res){
    res.running = running_;
    return true;
}

// Store Tag Data
void StoreTagData(string tag_data){
    stringstream tag_ss;
    tag_ss << tag_data;
    YAML::Parser parser(tag_ss);
    YAML::Node doc;
    parser.GetNextDocument(doc);
    
    for(YAML::Iterator field = doc.begin(); field != doc.end(); ++field){
        int index;
        field.first() >> index;
        
        const YAML::Node& value = field.second();
        for(YAML::Iterator sub_field = value.begin();
                sub_field != value.end(); ++sub_field){
            
            string key;
            sub_field.first() >> key;
            
            if(key == "size"){
                sub_field.second() >> tag_sizes_[index];
            }
        }
    }
}

void InitializeROSNode(int argc, char **argv)
{
    ros::init(argc, argv, "apriltags");
    node_ =  boost::make_shared<ros::NodeHandle>("~");
    image_ = boost::make_shared<image_transport::ImageTransport>(*node_);
}

int main(int argc, char **argv){

    InitializeROSNode(argc,argv);
    GetParameterValues();
    SetupPublisher();
    InitializeTags();

    if(viewer_){
        cvNamedWindow("AprilTags");
        cvStartWindowThread();
    }

    InitializeServices();
    StoreTagData(tag_data);
    ros::spin();

    //Destroying Stuff
    cvDestroyWindow("AprilTags");
    delete detector_;
    delete family_;

    return EXIT_SUCCESS;
}
