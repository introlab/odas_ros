/**
 * \file sound_tracking.h
 * \brief Rotate de base of the robot towards sound source
 * \author Sébastien Laniel
 * \version 0.1.1
 * \date 2017-05-05
 *
 * Depending on the sound_tracking monde, the robot will :
 *	THRESHOLD 	: 	Get the speech with most confidence and orientate to it.
 * 	WEIGHT 		:	Calculate a Weight for a speech location and orientate to it
 *	SLAVE		:	Obey a high-level reasoning
 */
#pragma once
#ifndef LIB_SOUND_TRACKING_H_
#define LIB_SOUND_TRACKING_H_

#include "ros/ros.h"
#include <tf/transform_datatypes.h>
#include "sound_tracking/lib/framework_common_utils.h"
#include "sound_tracking/lib/base_control.h"
#include "sound_tracking/lib/mode_threshold.h"
#include "sound_tracking/lib/source.h"
#include "sound_tracking/lib/filter.h"
#include "sound_tracking/lib/spatial_filter.h"
#include "nav_msgs/Odometry.h"
#include "odas_msgs/Tracked_sources.h"
#include "std_msgs/String.h"

using namespace soundTracking;

// SUBSCRIBERS | PUBLISHERS //
ros::Subscriber sub_odom;			// Subscriber used to read desired velocities by the manual cmd module
ros::Subscriber sub_odas;			// Subscriber used to read gamepad enable button
ros::Publisher  cmd_vel_pub;		// Publisher used to write desired velocities to the base

// TIMERS //
ros::Timer timerPublishCmdVel;		// Timer ti publish the commands at 10 Hz

// ROS PARAMETERS //
int baseFrequence;
bool print;
std::string NODE_NAME;

// GLOBAL VARIABLES //
std::string DEFAULT_NODE_NAME = "sound_tracking";
base_control * ptr_base_control;
spatial_filter * ptr_spatial_filter;
bool active;
int filter;


double robotYaw;


// PROTOTYPES //
void rosparam_read(ros::NodeHandle * ptr_n);
void rostopic_sub(ros::NodeHandle * ptr_n);
void rostopic_pub(ros::NodeHandle * ptr_n);
void rostimer_init(ros::NodeHandle * ptr_n);
void rosvariable_init(void);
void rosshutdown(void);
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg_);
void odasCallback(const odas_msgs::Tracked_sources::ConstPtr& msg_);
void timerPublishCmdVelProcess(const ros::TimerEvent&);

#endif
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
