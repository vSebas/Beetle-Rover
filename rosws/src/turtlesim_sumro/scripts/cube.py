from geometry_msgs.msg import Point32

class Cube(object):
    """ Represents a cube in the map """
    
    def __init__(self, number):
        self._number = number
        self._xpos = None
        self._ypos = None
    
    @property
    def number(self):
        return self._number

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

        # We asumme each cube is 20 cm
        offset = 0.002
        # If the cube is not a target we
        # should "inflate" it's size
        # so that it includes it's forbidden zone (50 cm)
        if not self.is_target:
            offset += 0.005

        v1 = Point32(x=self.xpos-offset, y=self.ypos+offset)
        v2 = Point32(x=self.xpost+offset, y=self.ypos+offset)
        v3 = Point32(x=self.xpos+offset, y=self.ypos-offset)
        v4 = Point32(x=self.xpos-offset, y=self.ypos-offset)
        
        return [v1, v2, v3, v4]
    
    @property
    def frame_id(self):
        """ Returns a string representing this cube's frame id """
        return 'C{:03d}'.format(self.number)
    
    @property
    def is_target(self):
        """ Returns True if this cube should be visited """
        return self.number % 2 != 0
    
    def __str__(self):
        return 'Cube (number={}, xpos={}, ypos={}, frame={})'.format(self.number, self.xpos, self.ypos, self.frame_id)