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
#ifndef LIB_SOUND_TRACKING_BASE_CONTROL_H_
#define LIB_SOUND_TRACKING_BASE_CONTROL_H_

#include <stdio.h>
#include "sound_tracking/lib/framework_common_utils.h"

namespace soundTracking 
{



struct msg
{
	bool send;
	double cmd;
};

class base_control
{
public:
	base_control() : robotYaw(0.0), targetYaw(0.0), greenZoneAngle(0.0), yellowZoneAngle(0.0), maxTurnSpeed(0.0), polarity(1), newTarget(false), angleCap(UTILS::PI/6)
	{ }
	base_control(const base_control &other) : robotYaw(0.0), targetYaw(0.0), greenZoneAngle(0.0), yellowZoneAngle(0.0), maxTurnSpeed(0.0), polarity(1), newTarget(false), angleCap(UTILS::PI/6)
	{ }
	base_control(double speed, double yellow, double green, int polarity) : angleCap(UTILS::PI/6)
	{ 
		this->yellowZoneAngle = UTILS::deg2rad(yellow);
		this->greenZoneAngle = UTILS::deg2rad(green);
		this->maxTurnSpeed = speed;
		this->polarity = polarity;
	}
	~base_control()
	{ }
	base_control& operator=(const base_control& other)	// Copy assignment
	{
		return *this;
	}
	void set_robotYaw(double robotYaw) 
	{
		this->robotYaw = robotYaw;
	}
	void set_targetYaw(double targetRelativeYaw)
	{
		newTarget = true;
		this->targetYaw = this->robotYaw + targetRelativeYaw;
		this->targetYaw = UTILS::ABS(this->targetYaw) > UTILS::PI ? this->targetYaw - UTILS::SIGN(this->targetYaw) * 2 * UTILS::PI : this->targetYaw;
	}
	void set_config(double greenZoneDegAngle, double yellowZoneDegAngle, double maxTurnSpeed, int polarity)
	{
		this->greenZoneAngle = UTILS::deg2rad(greenZoneDegAngle);
		this->yellowZoneAngle = UTILS::deg2rad(yellowZoneDegAngle);
		this->maxTurnSpeed = maxTurnSpeed;
		this->polarity = polarity;
	}
	msg do_work()
	{
		static bool moves = false;
		msg msg_ = {false, 0};

		double delta = this->targetYaw - this->robotYaw;
		delta = UTILS::ABS(delta) > UTILS::PI ? delta - UTILS::SIGN(delta) * 2 * UTILS::PI : delta;

		double proportionnel = UTILS::ABS(delta)/(angleCap);
		proportionnel = proportionnel > 1.0 ? 1.0 : proportionnel;

		if(!newTarget) return msg_;

		msg_.send = true;
		msg_.cmd = UTILS::SIGN(delta) * polarity * (maxTurnSpeed * proportionnel);

		/*if(UTILS::ABS(delta) > this->yellowZoneAngle) { 
			moves = true; 
		}

		msg_.send = moves;

		if(moves) {
			if(UTILS::ABS(delta) > this->greenZoneAngle) {
				msg_.cmd = UTILS::SIGN(delta) * polarity * (maxTurnSpeed * proportionnel);
			}
			else {
				moves = false;
				msg_.cmd = 0;
			}
		}*/

		return msg_;
	}

	double get_robotYaw() { return robotYaw; }
	double get_targetYaw() { return targetYaw; }

	void set_angleCap(double angle) { angleCap = angle; }
	
private:
	double robotYaw;
	double targetYaw;		// Absolute target Yaw
	double greenZoneAngle;	// Tolerance of ±radAngle that we consider the robot is oriented to the sound target
	double yellowZoneAngle;	// Tolerance of ±radAngle that we consider the robot must change its orientation (hysteresis = yellow - green)
	double maxTurnSpeed;
	int polarity;			// {1, -1} : Depending on robot odom
	bool newTarget;
	double angleCap;
};

} // namespace soundTracking 
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