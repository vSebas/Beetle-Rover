class Cube(object):
    """ Represents a cube in the map """
    
    def __init__(self, number):
        self._visited = False
        self._number = number
        self._xpos = None
        self._ypos = None
    
    @property
    def number(self):
        return self._number

    @property
    def visit_pending(self):
        """ Returns True if a visit to this cube is pending """
        return self.is_target and not self.visited
    
    @property
    def is_target(self):
        """ Returns True if this cube should be visited """
        return self.number % 2 != 0
    
    @property
    def visited(self):
        """ Returns True if this cube is marked as visited """
        return self._visited

    @property
    def frame_id(self):
        """ Returns a string representing this cube's frame id """
        return 'Cube{0}'.format(self.number)

    @property
    def xpos(self):
        return self._xpos
    
    @property
    def ypos(self):
        return self._ypos
    
    @xpos.setter
    def set_xpos(self, xpos):
        self._xpos = xpos
    
    @ypos.setter
    def set_ypos(self, ypos):
        self._ypos = ypos
    
    def visit(self):
        """ Marks cube as visited """
        self._visited = True

    def __str__(self):
        return 'Cube (number={}, visited={})'.format(self.number, self.visited)