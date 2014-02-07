#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

#include "../lib_apriltags/src/TagDetector.h"

#include <visualization_msgs/MarkerArray.h>

using namespace std;

#define DEFAULT_TAG_FAMILY string("Tag16h5")

class DetectAprilTags {
    
    ros::NodeHandle node_;
    image_transport::ImageTransport image_;
    ros::Publisher marker_publisher_;
    ros::ServiceServer detect_enable_;
    
    TagDetector* tag_detector_;
    
    int enabled;
public:    
    DetectAprilTags() :
            image_(node_) {
        
        // Get ROS parameters
        ros::NodeHandle param_node_handle("~");
        
        string camera_topic_name;
        string output_marker_list_topic_name;
        string tag_family_name;
        string enable_service_name;
        
        param_node_handle.param(
                "camera_base_topic", camera_topic_name, string("/Image"));
        param_node_handle.param(
                "output_marker_list_topic_name", output_marker_list_topic_name,
                string("/apriltag_markers"));
        param_node_handle.param(
                "enable_service_name", enable_service_name,
                string("/Enable"));
        param_node_handle.param(
                "tag_family", tag_family_name, DEFAULT_TAG_FAMILY);
        
        // Setup the tag detector
        TagFamily family(tag_family_name);
        tag_detector_ = new TagDetector(family);
        
        // Publisher
        marker_publisher_ = node_.advertise<visualization_msgs::MarkerArray>(
                output_marker_list_topic_name, 0);
        
        // Subscriber
        image_transport::CameraSubscriber sub = image_.subscribeCamera(
                camera_topic_name, 1, &DetectAprilTags::ImageCallback, this);
        
        //detect_enable_ = 
        
    }
    
    ~DetectAprilTags(){
        delete tag_detector_;
    }
    
    void ImageCallback(
            const sensor_msgs::ImageConstPtr& msg,
            const sensor_msgs::CameraInfoConstPtr& camera_info)
    {
        
        cout << "I can see!" << endl;
        
        sensor_msgs::CvBridge bridge;
        Mat subscribed_image;
        try{
            subscribed_image = bridge.imgMsgToCv(msg, "bgr8");
            cvShowImage("AprilTags", subscribed_image);
        }
        catch(sensor_msgs::CvBridgeException& e){
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                    msg->encoding.c_str());
        }
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "apriltags");
    
    cvNamedWindow("AprilTags");
    cvStartWindowThread();
    
    DetectAprilTags detector;
    ros::spin();
    
    cvDestroyWindow("AprilTags");
    
    return EXIT_SUCCESS;
    
    /*
    ros::NodeHandle node;
    */
    
    /*
    image_transport::ImageTransport image_transport(node);
    image_transport::Subscriber sub = 
            image_transport.subscribe("/Image", 1, imageCallback);
    */
    /*
    ros::spin();
    */
}
