/**
 * \file filter.h
 * \brief Filter parameters
 * \author Sébastien Laniel
 * \version 0.1.1
 * \date 2017-05-05
 *
 * Lowpass, highpass or bandpass filters
 */
#pragma once
#ifndef LIB_SOUND_TRACKING_FILTER_H_
#define LIB_SOUND_TRACKING_FILTER_H_

namespace framework
{
	enum filterType
	{
		NOFILTER = 0,
		LOWPASS,
		HIGHPASS,
		BANDPASS,
		BANDREJECT
	};
	
	template <class T>
	T lowpass(T x, T x0, T y, T zero = 0){
		return (x > x0 ? zero : y);
	}

	template <class T>
	T highpass(T x, T x0, T y, T zero = 0){
		return (x < x0 ? zero : y);
	}

	template <class T>
	T bandpass(T x, T x1, T x2, T y, T zero = 0){
		return ((x < x1 || x > x2) ? zero : y);
	}

	template <class T>
	T bandreject(T x, T x1, T x2, T y, T zero = 0){
		return ((x > x1 && x < x2) ? zero : y);
	}

	bool allowEveryNtimes(int N, int cpt, int * ptr_cpt = NULL) {
		bool res = false;

		if(++cpt >= N){
			cpt = 0;
			res = true;
		}

		if(ptr_cpt != NULL) 
			*ptr_cpt = cpt;
		
		return res;
	}
}

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