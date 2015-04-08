#!/usr/bin/env python

from naoqi_sensors.naoqi_camera import NaoqiCam

class PepperCam( NaoqiCam ):
  def __init__( self, node_name='pepper_camera' ):
    NaoqiCam.__init__( self, node_name )

  def init_config( self ):
    NaoqiCam.init_config( self )

  def extractParams( self, new_config ):
    pass

  def setParams( self, key_list ):
    pass
