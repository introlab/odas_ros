/**
 * \file sound_tracking.cpp
 * \brief Rotate de base of the robot towards sound source
 * \author Sébastien Laniel
 * \version 0.1.1
 * \date 2017-05-05
 *
 * Depending on the sound_tracking monde, the robot will :
 *	THRESHOLD 	: 	Get the speech with most confidence and orientate to it.
 * 	WEIGHT 		:	Calculate a Weight for a speech location and orientate to it
 *
 *	SLAVE		:	Obey a high-level reasoning
 */
#include "sound_tracking/sound_tracking.h"

using namespace soundTracking;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sound_tracking");
	ros::NodeHandle n;

	rosparam_read(&n);
	rostopic_sub(&n);
	rostopic_pub(&n);
	rostimer_init(&n);

	rosvariable_init();

	if(print) ROS_INFO_STREAM(NODE_NAME << " : ***************OUTPUT SCREEN ENABLE***************");

	ros::spin();
	
	rosshutdown();

	return 0;
}

void rosparam_read(ros::NodeHandle * ptr_n){
	ptr_n->param("sound_tracking/baseFrequence", baseFrequence, 0);
	ptr_n->param("sound_tracking/NODE_NAME", NODE_NAME, DEFAULT_NODE_NAME);
	ptr_n->param("sound_tracking/print", print, false);

	//Reading spatial filter configs
	double A, B;
	int type;
	ptr_n->param("sound_tracking/spatial_filter/A", A, 0.0);
	ptr_n->param("sound_tracking/spatial_filter/B", B, 0.0);
	ptr_n->param("sound_tracking/spatial_filter/type", type, 0);
	ptr_spatial_filter = new spatial_filter(A, B, (enum framework::filterType) type);

	// Reading base_control configs
	double yellowZone, greenZone, speed;
	int polarity;
	ptr_n->param("sound_tracking/base_control/polarity", polarity, 1);
	ptr_n->param("sound_tracking/base_control/speed", speed, 0.0);
	ptr_n->param("sound_tracking/base_control/yellowZone", yellowZone, 0.0);
	ptr_n->param("sound_tracking/base_control/greenZone", greenZone, 0.0);
	ptr_base_control = new base_control(speed, yellowZone, greenZone, polarity);
}

void rostopic_sub(ros::NodeHandle * ptr_n){
	sub_odom = ptr_n->subscribe<nav_msgs::Odometry>("odom", 1, odomCallback);
	sub_odas = ptr_n->subscribe<odas_msgs::Tracked_sources>("odas", 1, odasCallback);

	return;
}

void rostopic_pub(ros::NodeHandle * ptr_n){
	cmd_vel_pub = ptr_n->advertise<geometry_msgs::Twist>("cmd_vel", 1);

	return;
}

void rostimer_init(ros::NodeHandle * ptr_n) {
	double duration = baseFrequence != 0 ? 1.0/baseFrequence : 1.0;

	timerPublishCmdVel = ptr_n->createTimer(ros::Duration(duration), timerPublishCmdVelProcess);
	if(baseFrequence == 0) timerPublishCmdVel.stop();

	return;
}

void rosshutdown(void){
	sub_odas.shutdown();
	sub_odom.shutdown();
	cmd_vel_pub.shutdown();
	timerPublishCmdVel.stop();
	delete ptr_spatial_filter;
	delete ptr_base_control;

	ros::shutdown();
}

void rosvariable_init(void){

	return;
}

void timerPublishCmdVelProcess(const ros::TimerEvent&) 
{
	msg msg_ = ptr_base_control->do_work();
	if (print) ROS_INFO_STREAM("ROBOT YAW =  " << UTILS::rad2deg(ptr_base_control->get_robotYaw()) << "  TARGET YAW " << UTILS::rad2deg(ptr_base_control->get_targetYaw()));

	if(msg_.send){
		geometry_msgs::Twist cmd_vel;
		cmd_vel.angular.z = msg_.cmd;
		cmd_vel_pub.publish(cmd_vel);
		//

		if(cmd_vel.angular.z < 0.1 && cmd_vel.angular.z > -0.1){
            cmd_vel.angular.z = 0;
            cmd_vel_pub.publish(cmd_vel);
		}
		else if(cmd_vel.angular.z < 1 && cmd_vel.angular.z > -1){
            cmd_vel_pub.publish(cmd_vel);
        }
	}
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_)
{
	//ROS_INFO_STREAM("odom callback");
	double roll, pitch, yaw;
	tf::Quaternion q(odom_->pose.pose.orientation.x, odom_->pose.pose.orientation.y, odom_->pose.pose.orientation.z, odom_->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);
	ptr_base_control->set_robotYaw(yaw);
}

void odasCallback(const odas_msgs::Tracked_sources::ConstPtr& audio_)
{
	//ROS_INFO_STREAM("odas callback");
	static ros::Time timestamp = ros::Time::now();
	ros::Time timenow = ros::Time::now();
	ros::Duration _diff = timenow - timestamp;
	double diff = ((double)(_diff.sec * 1e9 + _diff.nsec)) / 1e9;

	for (int i=0; i < audio_->sources.size(); ++i) {
		if(audio_->sources[i].speech && diff > 0.5) {
			source* ptr_source = new source(audio_->sources[i].x, audio_->sources[i].y, audio_->sources[i].z, audio_->sources[i].activity);
			if(ptr_source->get_confidence() > 0.8)  {
				ptr_spatial_filter->filterPitch(ptr_source);
				timestamp = ros::Time::now();
				ptr_base_control->set_targetYaw(ptr_source->get_yaw());
			}
		}
	}
}


/*
Copyright (c) 2017, AGE-WELL and the Université de Sherbrooke
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
* Neither the name of the Université de Sherbrooke, AGE-WELL nor the
  names of its contributors may be used to endorse or promote products
  derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AGE-WELL AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL AGE-WELL AND CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
