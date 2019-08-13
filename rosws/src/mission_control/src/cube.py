from geometry_msgs.msg import Point32

# Cube size in meters
CUBE_SIZE = 0.20
# Approach zone size in meters
APPROACH_ZONE_SIZE = 0.30
# Forbidden zone in meters
FORBIDDEN_ZONE_SIZE = 0.5

class Cube(object):
    """ Represents a cube in the map """
    
    def __init__(self, number):
        self._number = number
        self._xpos = None
        self._ypos = None
        self._orientation = None
    
    @property
    def number(self):
        return self._number
    
    @property
    def orientation(self):
        return self._orientation
    
    @orientation.setter
    def orientation(self, orientation):
        self._orientation = orientation

    @property
    def xpos(self):
        return self._xpos
    
    @property
    def ypos(self):
        return self._ypos
    
    @xpos.setter
    def xpos(self, xpos):
        self._xpos = xpos
    
    @ypos.setter
    def ypos(self, ypos):
        self._ypos = ypos
    
    @property
    def approach_zone_xpos(self):
        return self.xpos + CUBE_SIZE + APPROACH_ZONE_SIZE/2.0

    @property
    def approach_zone_ypos(self):
        return self.ypos + CUBE_SIZE + APPROACH_ZONE_SIZE/2.0

    @property
    def bounding_box(self):
        """
        * = offset

        v1 = (x-offset, y+offset)
        v2 = (x+offset, y+offset)
        v3 = (x+offset, y-offset)
        v4 = (x-offset, y-offset)

        ^   Bounding box:
        |   v1----------v2
        |   |           |
        |   |   (x,y) * *
        |   |    *      |
        |   v4---*------v3
        |------------------------>x
        |
        |
        y
        
        """

        offset = CUBE_SIZE
        # If the cube is not a target we
        # should "inflate" it's size
        # so that it includes it's forbidden zone
        if not self.is_target:
            offset += FORBIDDEN_ZONE_SIZE

        v1 = Point32(x=self.xpos-offset, y=self.ypos+offset)
        v2 = Point32(x=self.xpos+offset, y=self.ypos+offset)
        v3 = Point32(x=self.xpos+offset, y=self.ypos-offset)
        v4 = Point32(x=self.xpos-offset, y=self.ypos-offset)
        
        return [v1, v2, v3, v4]
    
    @property
    def frame_id(self):
        """ Returns a string representing this cube's frame id """
        return 'cube{:03d}'.format(self.number)
    
    @property
    def is_target(self):
        """ Returns True if this cube should be visited """
        return self.number % 2 != 0
    
    def __str__(self):
        return 'Cube (number={}, xpos={}, ypos={}, frame={})'.format(self.number, self.xpos, self.ypos, self.frame_id)