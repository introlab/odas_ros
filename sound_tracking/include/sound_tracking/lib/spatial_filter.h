/**
 * \file spatial_filter.h
 * \brief BeamFormer that blocks sound source from a certain range angle
 * \author Sébastien Laniel
 * \version 0.1.1
 * \date 2017-05-05
 *
 * Lowpass, highpass or bandpass filters
 */
#pragma once
#ifndef LIB_SOUND_TRACKING_SPATIAL_FILTER_H_
#define LIB_SOUND_TRACKING_SPATIAL_FILTER_H_

#include <stdio.h>
#include "sound_tracking/lib/framework_common_utils.h"
#include "sound_tracking/lib/source.h"
#include "sound_tracking/lib/filter.h"

namespace soundTracking 
{

class spatial_filter
{
public:
	spatial_filter() : angleA(0), angleB(0), type(framework::NOFILTER)
	{ }
	spatial_filter(const spatial_filter &other) : angleA(0), angleB(0), type(framework::NOFILTER)
	{ }
	spatial_filter(double A, double B, enum framework::filterType type) : angleA(0), angleB(0), type(framework::NOFILTER)
	{
		this->angleA = UTILS::deg2rad(A);
		this->angleB = UTILS::deg2rad(B);
		this->type = type; 
	}
	~spatial_filter()
	{ }
	spatial_filter& operator=(const spatial_filter& other)	// Copy assignment
	{
		return *this;
	}
	void filterPitch(source* source){
		switch(type)
		{
			case framework::LOWPASS:
				source->set_confidence(framework::lowpass(source->get_pitch(), angleA, source->get_confidence()));
			break;

			case framework::HIGHPASS:
				source->set_confidence(framework::highpass(source->get_pitch(), angleA, source->get_confidence()));
			break;

			case framework::BANDPASS:
				if(angleA > angleB) return;
				source->set_confidence(framework::bandpass(source->get_pitch(), angleA, angleB, source->get_confidence()));
			break;

			case framework::BANDREJECT:
				if(angleA > angleB) return;
				source->set_confidence(framework::bandreject(source->get_pitch(), angleA, angleB, source->get_confidence()));
			break;

			case framework::NOFILTER:
			default:
			break;
		}
	}
protected:
	double angleA, angleB;
	enum framework::filterType type;
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