import rospy
import tf

class Cube(object):
    def __init__(self, number, name):
        self.number = number
        self.name = name
        self.position = []
        self.orientation = []



class GoalMaker(object):
    def __init__(self):
        rospy.init_node('TestNode')
        self.cube1 = Cube(1, "Cube1")
        self.cube2 = Cube(1, "Cube2")
        self.cubes = {}
        self.cubes["good"]=[]
        self.cubes["bad"] = []
        self.cubes["good"].append(self.cube1)
        self.cubes["bad"].append(self.cube2)
        self.transformer = tf.TransformListener()
        self.rate = rospy.Rate(10.0)
    def run (self):
        while not rospy.is_shutdown():
            for key in self.cubes.keys():
                for cube in self.cubes[key]:
                    try:
                        (cube.position,cube.orientation) = self.transformer.lookupTransform('/'+cube.name, '/world', rospy.Time(0))
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        continue
                
                print cube.position
                print cube.orientation
            self.rate.sleep()

if __name__ == '__main__':
    goal_maker = GoalMaker()
    goal_maker.run()