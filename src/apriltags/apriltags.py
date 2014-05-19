PACKAGE = 'apriltags'
import roslib; roslib.load_manifest(PACKAGE)
import roslaunch
import rospy
import srv
import atexit
import percy.detector as detector

'''
open_ids = set()

def cleanup_apriltags():
    for open_id in open_ids:
        print 'Cleaning up', open_id
        proxy = rospy.ServiceProxy('apriltags/stop', srv.Stop)
        proxy(open_id)

atexit.register(cleanup_apriltags)
'''

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
    
    '''
    def Start(self):
        is_open_proxy = rospy.ServiceProxy('apriltags/is_id_open', srv.IsIdOpen)
        already_open = is_open_proxy(self.start_id).open
        if not already_open:
            try:
                start_proxy = rospy.ServiceProxy('apriltags/start', srv.Start)
                self.start_id = start_proxy().id
                open_ids.add(self.start_id)
            except rospy.ServiceException:
                print "Service is not running!"
                raise
        
        return self.start_id
    
    def Stop(self):
        try:
            print 'a'
            #stop_proxy = rospy.ServiceProxy('apriltags/stop', srv.Stop)
            print self.start_id
            print 'b'
            closed = self.stop_proxy(self.start_id)
            print 'c'
            self.start_id = 0
            print 'd'
            open_ids.remove(self.start_id)
            return closed
        
        except rospy.ServiceException:
            print "Service is not running!"
            raise
    
    def StopAll(self):
        try:
            stop_proxy = rospy.ServiceProxy('apriltags/stop_all', srv.StopAll)
            stop_proxy()
        except rospy.ServiceException:
            print "Service is not running!"
            raise
    
    def IsRunning(self):
        try:
            running_proxy = rospy.ServiceProxy('apriltags/is_running',
                    srv.IsRunning)
            return running_proxy().running
        except rospy.ServiceException:
            print "Service is not running!"
            raise
    
    def AmIOpen(self):
        try:
            open_proxy = rospy.ServiceProxy('apriltags/is_id_open',
                    srv.IsIdOpen)
            return open_proxy(self.start_id).open
        except rospy.ServiceException:
            print "Service is not running!"
            raise
    
    def __enter__(self):
        return self
    
    def __exit__(self, ext_type, exc_value, traceback):
        print "EXIT!"
        self.Stop()
    
    #def __del__(self):
    #    print "__del__"
    #    self.Stop()
    '''
