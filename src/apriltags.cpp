#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/Marker.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
//#include <cv_bridge/CvBridge.h>
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

#include <apriltags/Start.h>
#include <apriltags/Stop.h>
#include <apriltags/StopAll.h>
#include <apriltags/IsRunning.h>

#define SMALL_TAG_SIZE 0.0358968
#define MED_TAG_SIZE 0.0630174
#define PAGE_TAG_SIZE 0.165

#define DEFAULT_TAG_FAMILY string("Tag36h11")

using namespace std;

class AprilTagsNode {
    // ROS interface
    ros::NodeHandle node_;
    image_transport::ImageTransport image_;
    ros::Publisher marker_publisher_;
    ros::TransportHints ros_transport_hints;
    image_transport::Subscriber image_subscriber;
    image_transport::TransportHints image_transport_hint;
    ros::Subscriber info_subscriber;
    ros::ServiceServer start_service_;
    ros::ServiceServer stop_service_;
    ros::ServiceServer stop_all_service_;
    ros::ServiceServer is_running_service_;
    
    // AprilTag parts
    TagFamily* family_;
    TagDetector* detector_;
    string tag_family_name_;
    sensor_msgs::CameraInfo camera_info_;
    
    // Settings and local information
    int viewer_;
    boost::unordered_map<size_t, double> tag_sizes_;
    double default_tag_size_;
    string frame_;
    bool running_;
    boost::unordered_set<int> open_ids_;
    int next_open_id_;

public:
    // Constructor
    AprilTagsNode() : node_("~"),
                      image_(node_),
                      ros_transport_hints(ros::TransportHints().tcpNoDelay()),
                      image_transport_hint(image_transport::TransportHints(
                            "raw", ros_transport_hints, node_,
                            "image_transport")){
        
        string camera_topic_name = "/Image";
        string output_marker_list_topic_name = "marker_array";
        string tag_data;
        
        // Set AprilTag options
        TagDetectorParams tag_params;
        tag_params.newQuadAlgorithm = 1;
        
        // Get parameters
        node_.param("/viewer", viewer_, 1);
        node_.param("/tag_family", tag_family_name_, DEFAULT_TAG_FAMILY);
        node_.param("/tag_data", tag_data, string(""));
        node_.param("/default_tag_size", default_tag_size_, SMALL_TAG_SIZE);
        node_.param("/tf_frame", frame_, string("/prosilica_cam"));
        
        // Initialize the tag family and detector
        family_ = new TagFamily(tag_family_name_);
        detector_ = new TagDetector(*family_, tag_params);
        
        // Start the viewer if speficified
        if(viewer_){
            cvNamedWindow("AprilTags");
            cvStartWindowThread();
        }
        
        // Publisher
        marker_publisher_ = node_.advertise<visualization_msgs::MarkerArray>(
                output_marker_list_topic_name, 1);
        
        // Subscribers
        image_subscriber = image_.subscribe(
                camera_topic_name, 1, &AprilTagsNode::ImageCallback, this,
                image_transport_hint);
        info_subscriber = node_.subscribe(
                "/camera_info", 10, &AprilTagsNode::InfoCallback, this);
        
        // Start/Stop Services
        running_ = false;
        start_service_ = node_.advertiseService(
                "start", &AprilTagsNode::StartService, this);
        stop_service_ = node_.advertiseService(
                "stop", &AprilTagsNode::StopService, this);
        stop_all_service_ = node_.advertiseService(
                "stop_all", &AprilTagsNode::StopAllService, this);
        is_running_service_ = node_.advertiseService(
                "is_running", &AprilTagsNode::IsRunningService, this);
        next_open_id_ = 1;
        
        // Store tag data
        StoreTagData(tag_data);
    }
    
    // Destructor
    ~AprilTagsNode(){
        cvDestroyWindow("AprilTags");
        delete detector_;
        delete family_;
    }
    
    // Start Service
    bool StartService(apriltags::Start::Request &req,
                      apriltags::Start::Response &res){
        res.id = next_open_id_;
        running_ = true;
        ++next_open_id_;
        open_ids_.insert(res.id);
        return true;
    }
    
    // Stop Service
    bool StopService(apriltags::Stop::Request &req,
                     apriltags::Stop::Response &res){
        open_ids_.erase(req.id);
        size_t num_open_ = open_ids_.size();
        res.closed = 0;
        if(num_open_ == 0){
            running_ = false;
            res.closed = 1;
        }
        return true;
    }
    
    // Stop All Service
    bool StopAllService(apriltags::StopAll::Request &req,
                        apriltags::StopAll::Response &res){
        open_ids_.clear();
        running_ = false;
        next_open_id_ = 1;
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
    
    // Callback for camera info
    void InfoCallback(
            const sensor_msgs::CameraInfoConstPtr& camera_info){
        if(running_){
            camera_info_ = (*camera_info);
        }
    }
    
    // Callback for image data
    void ImageCallback(
            const sensor_msgs::ImageConstPtr& msg )//,
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
        /*
        image_pts.push_back(cv::Point2f(detection.p[0].x, detection.p[0].y));
        image_pts.push_back(cv::Point2f(detection.p[1].x, detection.p[1].y));
        image_pts.push_back(cv::Point2f(detection.p[2].x, detection.p[2].y));
        image_pts.push_back(cv::Point2f(detection.p[3].x, detection.p[3].y));
        */
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
};

int main(int argc, char **argv){
    ros::init(argc, argv, "apriltags");
    
    AprilTagsNode detector;
    ros::spin();
    
    return EXIT_SUCCESS;
}
