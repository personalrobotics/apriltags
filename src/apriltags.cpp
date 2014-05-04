#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/Marker.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

//#include <src/TagDetectorParams.h>
#include <src/TagDetector.h>
#include <src/TagFamily.h>

/*
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag16h5.h>
#include <AprilTags/Tag25h7.h>
#include <AprilTags/Tag25h9.h>
#include <AprilTags/Tag36h9.h>
#include <AprilTags/Tag36h11.h>
*/

#include <visualization_msgs/MarkerArray.h>
#include "yaml-cpp/yaml.h"
#include <sstream>
#include <fstream>

#include <boost/unordered_map.hpp>
//
using namespace std;

//0.0378968; // (~1.5" tags)
// (~2.5" tags)
#define SMALL_TAG_SIZE 0.0358968
#define MED_TAG_SIZE 0.0630174
#define PAGE_TAG_SIZE 0.165

#define DEFAULT_TAG_FAMILY string("36h11")

class AprilTagsNode {
    ros::NodeHandle node_;
    image_transport::ImageTransport image_;
    ros::Publisher marker_publisher_;
    ros::ServiceServer detect_enable_;
    
    /*
    AprilTags::TagDetector* tag_detector_;
    AprilTags::TagCodes tag_codes_;
    */
    TagFamily* family_;
    TagDetector* detector_;
    
    string tag_family_name_;
    sensor_msgs::CameraInfo camera_info_;
    
    int viewer_;
    boost::unordered_map<size_t, double> tag_sizes_;
    double default_tag_size_;
    string frame_;
    
    ros::Subscriber image_subscriber;
    ros::Subscriber info_subscriber;
    
    int enabled_;

public:
    AprilTagsNode() : node_("~"),
                      image_(node_){
        
        string camera_topic_name = "/Image";
        string output_marker_list_topic_name = "/marker_array";
        string enable_service_name = "/Enable";
        string tag_data;
        
        // Get apriltags options
        TagDetectorParams tag_params;
        
        // Get Parameters
        node_.param("/viewer", viewer_, 0);
        node_.param("/tag_family", tag_family_name_, DEFAULT_TAG_FAMILY);
        family_ = new TagFamily(tag_family_name_);
        detector_ = new TagDetector(*family_, tag_params);
        
        node_.param("/tag_data", tag_data, string(""));
        
        node_.param("/default_tag_size", default_tag_size_, SMALL_TAG_SIZE);
        node_.param("/tf_frame", frame_, string("/head_kinect_rgb_frame"));
        
        // Start the viewer if speficified
        if(viewer_){
            cvNamedWindow("AprilTags");
            cvStartWindowThread();
        }
    
        // Tag Detector
        //SetTagCodeFromString(tag_family_name_);
        
        //tag_detector_ = new AprilTags::TagDetector(tag_codes_);
        
        // Publisher
        marker_publisher_ = node_.advertise<visualization_msgs::MarkerArray>(
                output_marker_list_topic_name, 0);
        
        // Subscriber
        //image_transport::CameraSubscriber sub = image_.subscribeCamera(
        //        camera_topic_name, 1, &AprilTagsNode::ImageCallback, this);
        
        image_subscriber = node_.subscribe(camera_topic_name, 10, &AprilTagsNode::ImageCallback, this);
        
        //image_transport::Subscriber sub = image_.subscribe(
        //        camera_topic_name, 1, &AprilTagsNode::ImageCallback, this);
        
        info_subscriber = node_.subscribe("/camera_info", 10, &AprilTagsNode::InfoCallback, this);
        
        // Store Tag Sizes
        StoreTagSizes(tag_data);
    }
    
    ~AprilTagsNode(){
        cvDestroyWindow("AprilTags");
        //delete tag_detector_;
        delete detector_;
        delete family_;
    }
    
    void StoreTagSizes(string tag_data){
        stringstream tag_ss;
        tag_ss << tag_data;
        YAML::Parser parser(tag_ss);
        YAML::Node doc;
        parser.GetNextDocument(doc);
        
        for(YAML::Iterator field = doc.begin(); field != doc.end(); ++field){
            //string index_str;
            //field.first() >> index_str;
            //size_t index = atoi(index_str.c_str());
            
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
    
    void InfoCallback(
            const sensor_msgs::CameraInfoConstPtr& camera_info){
        camera_info_ = (*camera_info);
    }
    
    void ImageCallback(
            const sensor_msgs::ImageConstPtr& msg )//,
           // const sensor_msgs::CameraInfoConstPtr& camera_info)
    {
        sensor_msgs::CvBridge bridge;
        cv::Mat subscribed_image;
        try{
            subscribed_image = bridge.imgMsgToCv(msg, "bgr8");
        }
        catch(sensor_msgs::CvBridgeException& e){
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                      msg->encoding.c_str());
            return;
        }
        
        cv::Mat subscribed_gray;
        cv::cvtColor(subscribed_image, subscribed_gray, CV_BGR2GRAY);
        //vector<AprilTags::TagDetection> detections =
        //        tag_detector_->extractTags(subscribed_gray);
        visualization_msgs::MarkerArray marker_transforms;
        
        /*
        for(unsigned int i = 0; i < detections.size(); ++i){
            double tag_size = default_tag_size_;
            boost::unordered_map<size_t, double>::iterator tag_size_it =
                    tag_sizes_.find(detections[i].id);
            
            if(tag_size_it != tag_sizes_.end()){
                tag_size = (*tag_size_it).second; 
            }
            
            if(viewer_){
                detections[i].draw(subscribed_image);
            }
            
            Eigen::Matrix4d pose;
            pose = detections[i].getRelativeTransform(
                    tag_size,
                    (camera_info_).K[0], (camera_info_).K[4],
                    (camera_info_).K[2], (camera_info_).K[5]);
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
            marker_transform.scale.x = tag_size;
            marker_transform.scale.y = tag_size;
            marker_transform.scale.z = 0.05 * tag_size;
            marker_transform.color.r = 0.5;
            marker_transform.color.g = 0.5;
            marker_transform.color.b = 0.5;
            marker_transform.color.a = 1.0;
            marker_transforms.markers.push_back(marker_transform);
        }
        */
        marker_publisher_.publish(marker_transforms);
        
        if(viewer_){
            cv::imshow("AprilTags", subscribed_image);
        }
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "apriltags");
    
    AprilTagsNode detector;
    ros::spin();
    
    return EXIT_SUCCESS;
}
