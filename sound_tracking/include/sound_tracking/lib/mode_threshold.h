/**
 * \file mode_treshold.h
 * \brief Sound Tracking Mode Threshold
 * \author Sébastien Laniel
 * \version 0.1.1
 * \date 2017-05-05
 *
 *	THRESHOLD 	: 	Get the speech with most confidence and orientate to it.
 */
 #pragma once
#ifndef LIB_SOUND_TRACKING_MODE_THRESHOLD_H_
#define LIB_SOUND_TRACKING_MODE_THRESHOLD_H_

#include <stdio.h>
#include "sound_tracking/lib/source.h"

namespace soundTracking 
{

class threshold
{
	public:
		threshold()	// Allocate
			: mainTarget(new source()), newTarget(false)
		{ }
		threshold(const threshold& other) // Copy Constructor
			: mainTarget(new source(*(other.mainTarget))), newTarget(false)
		{ }
		threshold& operator=(const threshold& other)	// Copy assignment
		{
			if (&other != this) {
				delete mainTarget;
				mainTarget = NULL;
				mainTarget = new source(*(other.mainTarget));
			}
			return *this;
		}
		~threshold()	// Deallocate
		{
			delete mainTarget;
		}
		std::string to_string()
		{
			return "Main Target = " + mainTarget->to_string();
		}
		void do_work(source* sources, int count) 
		{
			double confidence = 0;
			for (int i = 0; i < count; ++i)
			{
				if(sources[i].get_confidence() > confidence){
					mainTarget = &sources[i];
					confidence = sources[i].get_confidence();
					newTarget = true;
				}
			}

		}
		bool isNewTarget() 
		{
			if(newTarget){
				newTarget = false; // Only read once as true
				return true;
			} 
			return false; 
		}
		double get_relativeYaw() { return mainTarget->get_yaw(); }

	private:
		source* mainTarget;
		bool newTarget;
};

}	// namespace soundTracking 
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