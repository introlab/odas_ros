#pragma once
#ifndef LIB_SOUND_TRACKING_SOURCE_H_
#define LIB_SOUND_TRACKING_SOURCE_H_

#include <stdio.h>
#include <iostream>
#include <string>
#include <math.h>
#include "sound_tracking/lib/framework_common_utils.h"

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

namespace soundTracking 
{

class target{
	public:
		target() : yaw(0), pitch(0), confidence(0), weight(0), id(0) {}
		double yaw;
		double pitch;
		double confidence;
		double weight;
		int id;
};
class source
{
	// *** Rules of three compliant ***
	public:
		void initAngle(double x, double y, double z){
			m_target->pitch = atan2(z,UTILS::ABS(x));
			m_target->yaw = atan2(y,x);
		}
		source()	// Allocate
			: m_target(new target())
		{ }
		source(double x, double y, double z)
			: m_target(new target())
		{
			initAngle(x,y,z);
			m_target->confidence = 0;
			m_target->weight = 0;
		}
		source(double x, double y, double z, double confidence)
			: m_target(new target())
		{
			initAngle(x,y,z);
			m_target->confidence = (m_target->pitch != 0 && m_target->yaw !=0) ? confidence : 0;
			m_target->weight = 0;
		}
		source(const source& other) // Copy Constructor
			: m_target(new target(*(other.m_target)))
		{ }
		source& operator=(const source& other)	// Copy assignment
		{
			if (&other != this) {
				delete m_target;
				m_target = NULL;
				m_target = new target(*(other.m_target));
			}
			return *this;
		}
		~source()	// Deallocate
		{
			delete m_target;
		}
		std::string to_string()
		{
			return "Source Yaw = " + patch::to_string(UTILS::rad2deg(m_target->yaw)) + "deg  Source Pitch = " + patch::to_string(UTILS::rad2deg(m_target->pitch)) + "deg  Source confidence = " + patch::to_string(m_target->confidence) +"\n";
		}

		double 	get_pitch() { return m_target->pitch; }
		void 	set_pitch(double pitch){ m_target->pitch = pitch; }

		double 	get_yaw() { return m_target->yaw; }
		void 	set_yaw(double yaw) { m_target->yaw = yaw; }
		
		double 	get_confidence() { return m_target->confidence; }
		void 	set_confidence(double confidence) { m_target->confidence = confidence; }
		
		double 	get_weight() { return m_target->weight; }
		void 	set_weight(double weight) { m_target->weight = weight; }

		int		get_id() { return m_target->id; }
		void	set_id(int id) { m_target->id = id; }

	private:
		target* m_target;
};
}	// namespace soundTracking 
#endif