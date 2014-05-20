#define SMALL_TAG_SIZE 0.0358968
#define MED_TAG_SIZE 0.0630174
#define PAGE_TAG_SIZE 0.165

#define DEFAULT_TAG_FAMILY string("Tag36h11")
using namespace std;
// ROS parts
ros::NodeHandlePtr node_;
boost::shared_ptr<image_transport::ImageTransport> image_;

ros::Publisher marker_publisher_;

image_transport::Subscriber image_subscriber;
ros::Subscriber info_subscriber;

ros::ServiceServer start_service_;
ros::ServiceServer stop_service_;
ros::ServiceServer stop_all_service_;
ros::ServiceServer is_running_service_;
ros::ServiceServer is_id_open_service_;
ros::ServiceServer running_ids_service_;

sensor_msgs::CameraInfo camera_info_;

// AprilTag parts
TagFamily* family_;
TagDetector* detector_;
string tag_family_name_;
TagDetectorParams tag_params;
string camera_topic_name = "/Image";
string output_marker_list_topic_name = "marker_array";
string tag_data;

// Settings and local information
int viewer_;
boost::unordered_map<size_t, double> tag_sizes_;
double default_tag_size_;
string frame_;
bool running_;

Eigen::Matrix4d GetDetectionTransform(TagDetection detection);
void InfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info);
void InfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info);
void ConnectCallback(const ros::SingleSubscriberPublisher& info);
void DisconnectCallback(const ros::SingleSubscriberPublisher& info);
void DisconnectHandler();
void GetParameterValues();
void SetupPublisher();
void InitializeTags();
void StoreTagData(string tag_data);
void InitializeROSNode();
