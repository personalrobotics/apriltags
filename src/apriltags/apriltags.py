PACKAGE = 'apriltags'
import roslib; roslib.load_manifest(PACKAGE)
import roslaunch
import rospy
import srv
import atexit
import percy.detector as detector

class AprilTags(detector.Detector):
    def __init__(self,
                 namespace='/apriltags',
                 subscribe_topic='/Image',
                 publish_topic='marker_array'):
        self.start_id = 0
        detector.Detector.__init__(self, namespace, subscribe_topic,
                publish_topic)
    
    def SetFamily(family_name):
        pass
