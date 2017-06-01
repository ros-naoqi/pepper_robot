#!/usr/bin/env python

# Copyright (C) 2014 Aldebaran Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#


#import ROS dependencies
import rospy
from sensor_msgs.msg import PointCloud2, PointField

# navigation test
from sensor_msgs.msg import LaserScan

#import PEPPER dependencies
from naoqi_driver.naoqi_node import NaoqiNode

#general
import copy
import math
import struct

#PEPPER specifications:
#https://community.aldebaran.com/doc/1-14/family/robots/laser_robot.html
class NaoqiLaser(NaoqiNode):

    # PEPPER laser specs
    # see https://community.aldebaran.com/doc/2-1/family/juliette_technical/laser_juliette.html#juliette-laser
    PEPPER_LASER_FREQ = 6                       # [hZ] --> check on that
    PEPPER_LASER_MIN_ANGLE = -0.523598776          # [rad]
    PEPPER_LASER_MAX_ANGLE = 0.523598776           # [radi]
    PEPPER_LASER_FOV = math.fabs(PEPPER_LASER_MIN_ANGLE)+math.fabs(PEPPER_LASER_MAX_ANGLE)

    PEPPER_LASER_MIN_RANGE = 0.1                   # [m] --> no spec given here
    PEPPER_LASER_MAX_RANGE = 5.0                   # [m] --> same here, 5m as quality guess

    # FRONT GROUND LASERS
    PEPPER_LASER_GROUND_SHOVEL_POINTS = 3
    PEPPER_LASER_GROUND_LEFT_POINTS = 1
    PEPPER_LASER_GROUND_RIGHT_POINTS = 1
    # SURROUNDING LASER
    PEPPER_LASER_SRD_POINTS = 15

    # memory key to fetch laser readings from
    # see memory key listing https://community.aldebaran.com/doc/2-1/family/juliette_technical/juliette_dcm/actuator_sensor_names.html#lasers
    # iterate over all segments e.g./SubDeviceList/Platform/LaserSensor/Front/Shovel/Seg01/X/Value
    PEPPER_MEM_KEY_GROUND_SHOVEL = 'Device/SubDeviceList/Platform/LaserSensor/Front/Shovel/'
    PEPPER_MEM_KEY_GROUND_LEFT = 'Device/SubDeviceList/Platform/LaserSensor/Front/Vertical/Left/'
    PEPPER_MEM_KEY_GROUND_RIGHT = 'Device/SubDeviceList/Platform/LaserSensor/Front/Vertical/Right/'
    PEPPER_MEM_KEY_SRD_FRONT = 'Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/'
    PEPPER_MEM_KEY_SRD_LEFT = 'Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/'
    PEPPER_MEM_KEY_SRD_RIGHT = 'Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/'

    # ROS params to check
    # acc. to spec: 40 kHz
    PARAM_LASER_RATE = '~laser_rate'
    PARAM_LASER_RATE_DEFAULT = PEPPER_LASER_FREQ

    # frame id to publish
    PARAM_LASER_SHOVEL_FRAME = '~laser_shovel_frame_id'
    PARAM_LASER_SHOVEL_FRAME_DEFAULT = 'ShovelLaser_frame'

    PARAM_LASER_GROUND_LEFT_FRAME = '~laser_ground_left_frame_id'
    PARAM_LASER_GROUND_LEFT_FRAME_DEFAULT = 'VerticalLeftLaser_frame'

    PARAM_LASER_GROUND_RIGHT_FRAME = '~laser_ground_right_frame_id'
    PARAM_LASER_GROUND_RIGHT_FRAME_DEFAULT = 'VerticalRightLaser_frame'

    PARAM_LASER_SRD_FRONT_FRAME = '~laser_srd_front_frame_id'
    PARAM_LASER_SRD_FRONT_FRAME_DEFAULT = 'SurroundingFrontLaser_frame'

    PARAM_LASER_SRD_LEFT_FRAME = '~laser_srd_left_frame_id'
    PARAM_LASER_SRD_LEFT_FRAME_DEFAULT = 'SurroundingLeftLaser_frame'

    PARAM_LASER_SRD_RIGHT_FRAME = '~laser_srd_right_frame_id'
    PARAM_LASER_SRD_RIGHT_FRAME_DEFAULT = 'SurroundingRightLaser_frame'

    TOPIC_LASER_SHOVEL = '~/laser/shovel/'
    TOPIC_LASER_GROUND_LEFT = '~/laser/ground_left/'
    TOPIC_LASER_GROUND_RIGHT = '~/laser/ground_right/'
    TOPIC_LASER_SRD_FRONT = '~/laser/srd_front/'
    TOPIC_LASER_SRD_LEFT = '~/laser/srd_left/'
    TOPIC_LASER_SRD_RIGHT = '~/laser/srd_right/'

    PEPPER_LASER_SUB_NAME = 'pepper_ros_laser'

    def __init__(self, pointcloud=True, laserscan=False):
        self.pointcloud = pointcloud
        self.laserscan = laserscan

        NaoqiNode.__init__(self, 'pepper_node')

        # ROS initialization;
        self.connectNaoQi()

        # default sensor rate: 25 Hz (50 is max, stresses Nao's CPU)
        self.laserRate = rospy.Rate(rospy.get_param(
                            self.PARAM_LASER_RATE,
                            self.PARAM_LASER_RATE_DEFAULT))

        self.laserShovelFrame = rospy.get_param(
                                    self.PARAM_LASER_SHOVEL_FRAME,
                                    self.PARAM_LASER_SHOVEL_FRAME_DEFAULT)
        self.laserGroundLeftFrame = rospy.get_param(
                                        self.PARAM_LASER_GROUND_LEFT_FRAME,
                                        self.PARAM_LASER_GROUND_LEFT_FRAME_DEFAULT)
        self.laserGroundRightFrame = rospy.get_param(
                                        self.PARAM_LASER_GROUND_RIGHT_FRAME,
                                        self.PARAM_LASER_GROUND_RIGHT_FRAME_DEFAULT)
        self.laserSRDFrontFrame = rospy.get_param(
                                        self.PARAM_LASER_SRD_FRONT_FRAME,
                                        self.PARAM_LASER_SRD_FRONT_FRAME_DEFAULT)
        self.laserSRDLeftFrame = rospy.get_param(
                                        self.PARAM_LASER_SRD_LEFT_FRAME,
                                        self.PARAM_LASER_SRD_LEFT_FRAME_DEFAULT)
        self.laserSRDRightFrame = rospy.get_param(
                                        self.PARAM_LASER_SRD_RIGHT_FRAME,
                                        self.PARAM_LASER_SRD_RIGHT_FRAME_DEFAULT)

        if self.pointcloud:
            self.pcShovelPublisher = rospy.Publisher(self.TOPIC_LASER_SHOVEL+'pointcloud', PointCloud2, queue_size=1)
            self.pcGroundLeftPublisher = rospy.Publisher(self.TOPIC_LASER_GROUND_LEFT+'pointcloud', PointCloud2, queue_size=1)
            self.pcGroundRightPublisher = rospy.Publisher(self.TOPIC_LASER_GROUND_RIGHT+'pointcloud', PointCloud2, queue_size=1)
            self.pcSRDFrontPublisher = rospy.Publisher(self.TOPIC_LASER_SRD_FRONT+'pointcloud', PointCloud2, queue_size=1)
            self.pcSRDLeftPublisher = rospy.Publisher(self.TOPIC_LASER_SRD_LEFT+'pointcloud', PointCloud2, queue_size=1)
            self.pcSRDRightPublisher = rospy.Publisher(self.TOPIC_LASER_SRD_RIGHT+'pointcloud', PointCloud2, queue_size=1)

        if self.laserscan:
            self.laserShovelPublisher = rospy.Publisher(self.TOPIC_LASER_SHOVEL+'scan', LaserScan, queue_size=1)
            self.laserGroundLeftPublisher = rospy.Publisher(self.TOPIC_LASER_GROUND_LEFT+'scan', LaserScan, queue_size=1)
            self.laserGroundRightPublisher = rospy.Publisher(self.TOPIC_LASER_GROUND_RIGHT+'scan', LaserScan, queue_size=1)
            self.laserSRDFrontPublisher = rospy.Publisher(self.TOPIC_LASER_SRD_FRONT+'scan', LaserScan, queue_size=1)
            self.laserSRDLeftPublisher = rospy.Publisher(self.TOPIC_LASER_SRD_LEFT+'scan', LaserScan, queue_size=1)
            self.laserSRDRightPublisher = rospy.Publisher(self.TOPIC_LASER_SRD_RIGHT+'scan', LaserScan, queue_size=1)

        self.laserSRDFrontPublisher_test = rospy.Publisher("~/pepper_navigation/front", LaserScan, queue_size=1)

    # (re-) connect to NaoQI:
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.laserProxy = self.get_proxy("ALLaser")
        self.memProxy = self.get_proxy("ALMemory")
        if self.laserProxy is None or self.memProxy is None:
            print "could not start either laser or memory proxy"
            exit(1)

    # fetch laser values
    ''' the idea here is to get the point xy
        calculate the length from the origin (range)
        assumption: field of view spanning from very min to very max
    '''
    def fetchLaserValues(self, keyPrefix, scanNum):
        ranges = []
        # traverse backwards
        tmp_array = []
        for i in xrange(scanNum, 0, -1):
            keyX = keyPrefix + 'Seg' + '%02d' % (i,) + '/X/Sensor/Value'
            keyY = keyPrefix + 'Seg' + '%02d' % (i,) + '/Y/Sensor/Value'
            tmp_array.append(keyX)
            tmp_array.append(keyY)
        memData = self.memProxy.getListData(tmp_array)
        for i in xrange(0, len(tmp_array), 2):
            x = memData[i]
            y = memData[i+1]
            ranges.append(math.sqrt(math.pow(x, 2.0) + math.pow(y, 2.0)))
        return ranges

    def fetchPCValues(self, keyPrefix, scanNum):
        scans = []
        for i in xrange(scanNum,0,-1):
            keyX = keyPrefix+'Seg'+'%02d'%(i,)+'/X/Sensor/Value'
            keyY = keyPrefix+'Seg'+'%02d'%(i,)+'/Y/Sensor/Value'
            x = self.memProxy.getData(keyX)
            y = self.memProxy.getData(keyY)
            scans.append(x)
            scans.append(y)
            scans.append(0.0)
        ba = struct.pack('%sf' %len(scans), *scans)
        return ba

    def createPointCloudMessage(self, frameID, keyPrefix, scanNum):
        pointCloudMsg = PointCloud2()
        pointCloudMsg.header.frame_id = frameID
        pointCloudMsg.header.stamp = rospy.Time.now()
        pointCloudMsg.height = 1
        pointCloudMsg.width = scanNum
        pointCloudMsg.is_dense = False
        pointCloudMsg.is_bigendian = False
        pointCloudMsg.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                                PointField('y', 4, PointField.FLOAT32, 1),
                                PointField('z', 8, PointField.FLOAT32, 1)]
        pointCloudMsg.point_step = 4*3
        pointCloudMsg.row_step = pointCloudMsg.point_step* pointCloudMsg.width
        pointCloudMsg.data = self.fetchLaserValues(keyPrefix, scanNum)
        return pointCloudMsg

    def createLaserMessage(self, frameID, keyPrefix, scanNum):
        laserScanMsg = LaserScan()
        laserScanMsg.header.frame_id = frameID
        laserScanMsg.angle_min = self.PEPPER_LASER_MIN_ANGLE
        laserScanMsg.angle_max = self.PEPPER_LASER_MAX_ANGLE
        laserScanMsg.angle_increment = self.PEPPER_LASER_FOV/scanNum
        laserScanMsg.range_min = self.PEPPER_LASER_MIN_RANGE
        laserScanMsg.range_max = self.PEPPER_LASER_MAX_RANGE
        return laserScanMsg

    # do it!
    def run(self):
        # start subscriber to laser sensor
        #self.laserProxy.subscribe(self.PEPPER_LASER_SUB_NAME)

        # we publish 6 laser messages in total
        # 1. shovel, 2. ground_left, 3. ground_right
        # 4. srd_front 5. srd_left 6. srd_right
        if self.pointcloud:
            shovelPC = self.createPointCloudMessage(
                                self.laserShovelFrame,
                                self.PEPPER_MEM_KEY_GROUND_SHOVEL,
                                self.PEPPER_LASER_GROUND_SHOVEL_POINTS )
            groundLeftPC = self.createPointCloudMessage(
                                self.laserGroundLeftFrame,
                                self.PEPPER_MEM_KEY_GROUND_LEFT,
                                self.PEPPER_LASER_GROUND_LEFT_POINTS )
            groundRightPC = self.createPointCloudMessage(
                                self.laserGroundRightFrame,
                                self.PEPPER_MEM_KEY_GROUND_RIGHT,
                                self.PEPPER_LASER_GROUND_RIGHT_POINTS )
            srdFrontPC = self.createPointCloudMessage(
                                self.laserSRDFrontFrame,
                                self.PEPPER_MEM_KEY_SRD_FRONT,
                                self.PEPPER_LASER_SRD_POINTS )
            srdLeftPC = self.createPointCloudMessage(
                                self.laserSRDLeftFrame,
                                self.PEPPER_MEM_KEY_SRD_LEFT,
                                self.PEPPER_LASER_SRD_POINTS )
            srdRightPC = self.createPointCloudMessage(
                                self.laserSRDRightFrame,
                                self.PEPPER_MEM_KEY_SRD_RIGHT,
                                self.PEPPER_LASER_SRD_POINTS )

        if self.laserscan:
            shovelScan = self.createLaserMessage(
                                self.laserShovelFrame,
                                self.PEPPER_MEM_KEY_GROUND_SHOVEL,
                                self.PEPPER_LASER_GROUND_SHOVEL_POINTS )
            groundLeftScan = self.createLaserMessage(
                                self.laserGroundLeftFrame,
                                self.PEPPER_MEM_KEY_GROUND_LEFT,
                                self.PEPPER_LASER_GROUND_LEFT_POINTS )
            groundRightScan = self.createLaserMessage(
                                self.laserGroundRightFrame,
                                self.PEPPER_MEM_KEY_GROUND_RIGHT,
                                self.PEPPER_LASER_GROUND_RIGHT_POINTS )
            srdFrontScan = self.createLaserMessage(
                                self.laserSRDFrontFrame,
                                self.PEPPER_MEM_KEY_SRD_FRONT,
                                self.PEPPER_LASER_SRD_POINTS )
            srdLeftScan = self.createLaserMessage(
                                self.laserSRDLeftFrame,
                                self.PEPPER_MEM_KEY_SRD_LEFT,
                                self.PEPPER_LASER_SRD_POINTS )
            srdRightScan = self.createLaserMessage(
                                self.laserSRDRightFrame,
                                self.PEPPER_MEM_KEY_SRD_RIGHT,
                                self.PEPPER_LASER_SRD_POINTS )

        while(self.is_looping()):

            if self.laserscan:
                # fetch values
                shovelScan.header.stamp = rospy.Time.now()
                shovelScan.ranges = self.fetchLaserValues(
                                            self.PEPPER_MEM_KEY_GROUND_SHOVEL,
                                            self.PEPPER_LASER_GROUND_SHOVEL_POINTS
                                            )

                groundLeftScan.header.stamp = rospy.Time.now()
                groundLeftScan.ranges = self.fetchLaserValues(
                                            self.PEPPER_MEM_KEY_GROUND_LEFT,
                                            self.PEPPER_LASER_GROUND_LEFT_POINTS
                                            )

                groundRightScan.header.stamp = rospy.Time.now()
                groundRightScan.ranges = self.fetchLaserValues(
                                            self.PEPPER_MEM_KEY_GROUND_RIGHT,
                                            self.PEPPER_LASER_GROUND_RIGHT_POINTS
                                            )

                srdFrontScan.header.stamp = rospy.Time.now()
                srdFrontScan.ranges = self.fetchLaserValues(
                                            self.PEPPER_MEM_KEY_SRD_FRONT,
                                            self.PEPPER_LASER_SRD_POINTS
                                            )

                srdLeftScan.header.stamp = rospy.Time.now()
                srdLeftScan.ranges = self.fetchLaserValues(
                                            self.PEPPER_MEM_KEY_SRD_LEFT,
                                            self.PEPPER_LASER_SRD_POINTS
                                            )

                srdRightScan.header.stamp = rospy.Time.now()
                srdRightScan.ranges = self.fetchLaserValues(
                                            self.PEPPER_MEM_KEY_SRD_RIGHT,
                                            self.PEPPER_LASER_SRD_POINTS
                                            )

                # publish messages
                self.laserShovelPublisher.publish(shovelScan)
                self.laserGroundLeftPublisher.publish(groundLeftScan)
                self.laserGroundRightPublisher.publish(groundRightScan)
                self.laserSRDFrontPublisher.publish(srdFrontScan)
                self.laserSRDLeftPublisher.publish(srdLeftScan)
                self.laserSRDRightPublisher.publish(srdRightScan)


            if self.pointcloud:
                # fetch values
                shovelPC.header.stamp = rospy.Time.now()
                shovelPC.data = self.fetchPCValues(
                                            self.PEPPER_MEM_KEY_GROUND_SHOVEL,
                                            self.PEPPER_LASER_GROUND_SHOVEL_POINTS
                                            )

                groundLeftPC.header.stamp = rospy.Time.now()
                groundLeftPC.data = self.fetchPCValues(
                                            self.PEPPER_MEM_KEY_GROUND_LEFT,
                                            self.PEPPER_LASER_GROUND_LEFT_POINTS
                                            )

                groundRightPC.header.stamp = rospy.Time.now()
                groundRightPC.data = self.fetchPCValues(
                                            self.PEPPER_MEM_KEY_GROUND_RIGHT,
                                            self.PEPPER_LASER_GROUND_RIGHT_POINTS
                                            )

                srdFrontPC.header.stamp = rospy.Time.now()
                srdFrontPC.data = self.fetchPCValues(
                                            self.PEPPER_MEM_KEY_SRD_FRONT,
                                            self.PEPPER_LASER_SRD_POINTS
                                            )

                srdLeftPC.header.stamp = rospy.Time.now()
                srdLeftPC.data = self.fetchPCValues(
                                            self.PEPPER_MEM_KEY_SRD_LEFT,
                                            self.PEPPER_LASER_SRD_POINTS
                                            )

                srdRightPC.header.stamp = rospy.Time.now()
                srdRightPC.data = self.fetchPCValues(
                                            self.PEPPER_MEM_KEY_SRD_RIGHT,
                                            self.PEPPER_LASER_SRD_POINTS
                                            )

                # publish messages
                self.pcShovelPublisher.publish(shovelPC)
                self.pcGroundLeftPublisher.publish(groundLeftPC)
                self.pcGroundRightPublisher.publish(groundRightPC)
                self.pcSRDFrontPublisher.publish(srdFrontPC)
                self.pcSRDLeftPublisher.publish(srdLeftPC)
                self.pcSRDRightPublisher.publish(srdRightPC)

            #sleep
            self.laserRate.sleep()


if __name__ == '__main__':
#    from optparse import OptionParser
#    parser = OptionParser()
#    parser.add_option("--ppointcloud", dest="ppointcloud", default=True)
#    parser.add_option("--plaser", dest="plaser", default=False)
#
#    (options, args) = parser.parse_args()
#    with_pc = options.ppointcloud
#    with_laser = options.plaser

    with_pc = True
    with_laser = True
    laser = NaoqiLaser(with_pc, with_laser)
    laser.start()

    rospy.spin()
    exit(0)
