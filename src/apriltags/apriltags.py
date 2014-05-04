PACKAGE = 'apriltags_swat'
import roslib; roslib.load_manifest(PACKAGE)
import roslaunch

visible_tags = None

class AprilTags(object):
    def __init__(self, namespace='/'):
        self.node = roslaunch.core.Node(
                PACKAGE, 'apriltags', namespace=namespace)
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()
        self.process = None
    
    def Start(self):
        if not self.IsRunning():
            self.process = launch.launch(node)
    
    def Stop(self):
        if self.IsRunning():
            self.process.stop()
    
    def IsRunning(self):
        return (self.process is not None) and self.process.is_alive()
