#!/usr/bin/env python

#
# ROS node to control Nao's walking engine (omniwalk and footsteps)
# This code is currently compatible to NaoQI version 1.6 or newer (latest
# tested: 1.12)
#
# Copyright 2009-2011 Armin Hornung & Stefan Osswald, University of Freiburg
# http://www.ros.org/wiki/nao
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    # Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    # Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    # Neither the name of the University of Freiburg nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import rospy

from naoqi_driver.naoqi_node import NaoqiNode

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D

from std_srvs.srv import Empty, EmptyResponse
from naoqi_bridge_msgs.srv import CmdPoseService, CmdVelService, CmdPoseServiceResponse, CmdVelServiceResponse, SetArmsEnabled, SetArmsEnabledResponse

class PepperWalker(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'pepper_walker')

        self.connectNaoQi()

        # other params
        self.maxHeadSpeed = rospy.get_param('~max_head_speed', 0.2)
        # initial stiffness (defaults to 0 so it doesn't strain the robot when no teleoperation is running)
        # set to 1.0 if you want to control the robot immediately
        initStiffness = rospy.get_param('~init_stiffness', 0.0)

        # TODO: parameterize
        if initStiffness > 0.0 and initStiffness <= 1.0:
            self.motionProxy.stiffnessInterpolation('Body', initStiffness, 0.5)

        # last: ROS subscriptions (after all vars are initialized)
        rospy.Subscriber("cmd_vel", Twist, self.handleCmdVel, queue_size=1)
        rospy.Subscriber("cmd_pose", Pose2D, self.handleTargetPose, queue_size=1)

        # Create ROS publisher for speech
        self.pub = rospy.Publisher("speech", String, latch = True)

        # ROS services (blocking functions)
        self.cmdPoseSrv = rospy.Service("cmd_pose_srv", CmdPoseService, self.handleTargetPoseService)
        self.cmdVelSrv = rospy.Service("cmd_vel_srv", CmdVelService, self.handleCmdVelService)
        self.stopWalkSrv = rospy.Service("stop_walk_srv", Empty, self.handleStopWalkSrv)
        self.setArmsEnabledSrv = rospy.Service("enable_arms_walking_srv", SetArmsEnabled, self.handleSetArmsEnabledSrv)

        self.say("Walker online")

        rospy.loginfo("nao_walker initialized")

    def connectNaoQi(self):
        '''(re-) connect to NaoQI'''
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.motionProxy = self.get_proxy("ALMotion")
        if self.motionProxy is None:
            exit(1)

    def stopWalk(self):
        """ Stops the current walking bahavior and blocks until the clearing is complete. """
        try:
            self.motionProxy.setWalkTargetVelocity(0.0, 0.0, 0.0, self.stepFrequency)
            self.motionProxy.waitUntilWalkIsFinished()


        except RuntimeError,e:
            print "An error has been caught"
            print e
            return False

        return True


    def say(self, text):
        self.pub.publish(text)

    def handleCmdVel(self, data):
        rospy.logdebug("Walk cmd_vel: %f %f %f, frequency %f", data.linear.x, data.linear.y, data.angular.z, self.stepFrequency)
        if data.linear.x != 0 or data.linear.y != 0 or data.angular.z != 0:
            self.gotoStartWalkPose()
        try:
            eps = 1e-3 # maybe 0,0,0 is a special command in motionProxy...
            if abs(data.linear.x)<eps and abs(data.linear.y)<eps and abs(data.angular.z)<eps:
                self.motionProxy.setWalkTargetVelocity(0,0,0,0.5)
            else:
                self.motionProxy.setWalkTargetVelocity(data.linear.x, data.linear.y, data.angular.z, self.stepFrequency, self.footGaitConfig)
        except RuntimeError,e:
            # We have to assume there's no NaoQI running anymore => exit!
            rospy.logerr("Exception caught in handleCmdVel:\n%s", e)
            rospy.signal_shutdown("No NaoQI available anymore")



    def handleCmdVelService(self, req):
        self.handleCmdVel(req.twist)
        return CmdVelServiceResponse()

    def handleTargetPose(self, data, post=True):
        """handles cmd_pose requests, walks to (x,y,theta) in robot coordinate system"""

        rospy.logdebug("Walk target_pose: %f %f %f", data.x,
                data.y, data.theta)

        self.gotoStartWalkPose()

        try:
            if post:
                self.motionProxy.post.walkTo(data.x, data.y, data.theta, self.footGaitConfig)
            else:
                self.motionProxy.walkTo(data.x, data.y, data.theta, self.footGaitConfig)
            return True
        except RuntimeError,e:
            rospy.logerr("Exception caught in handleTargetPose:\n%s", e)
            return False


    def handleTargetPoseService(self, req):
        """ do NOT use post"""
        if self.handleTargetPose(req.pose, False):
            return CmdPoseServiceResponse()
        else:
            return None

    def handleStopWalkSrv(self, req):
        if self.stopWalk():
            return EmptyResponse()
        else:
            return None

    def handleSetArmsEnabledSrv(self, req):
        self.motionProxy.setWalkArmsEnable(req.left_arm, req.right_arm)
        rospy.loginfo("Arms enabled during walk: left(%s) right(%s)" % (req.left_arm, req.right_arm))
        return SetArmsEnabledResponse()


if __name__ == '__main__':
    walker = PepperWalker()
    rospy.loginfo("pepper_walker running...")
    rospy.spin()
    rospy.loginfo("pepper_walker stopping...")
    walker.stopWalk()

    rospy.loginfo("pepper_walker stopped.")
    exit(0)
