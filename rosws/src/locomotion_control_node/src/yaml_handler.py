#!/usr/bin/python
# Import yaml files:
from yaml import load, dump
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper
import os
# Class for handling yaml file 
class YamlHandler(object):
    def __init__(self, config_file,*args, **kwargs):
        if "file_location" in kwargs:
            file_location = kwargs['file_location']
        else:
            file_location = os.path.abspath(os.path.dirname(__file__))
        self._file= load(open(file_location+'/'+config_file),Loader=Loader)

    def get_file(self):
        return self._file
    