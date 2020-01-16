#pragma once
#ifndef UTILS_H_
#define UTILS_H_

#include <string>
#include <sstream>
#include <iostream>
#include <stdint.h>

namespace UTILS {
	const double PI = 3.14159265359;
	static int _id = 0;

	template <class T>
	static T ABS(T a){
		return (a < 0 ? -a : a);
	}

	template <class T>
	static int SIGN(T a){
		return (a < 0 ? -1 : 1);
	}

	/*static inline std::ostream& operator<<(std::ostream& o, const HexCharStruct& hs)
	{
	    return (o << std::hex << (int)hs.c << std::dec);
	}

	static inline HexCharStruct hex(unsigned char _c)
	{
	    return HexCharStruct(_c);
	}*/

	static uint16_t Converter_toUint16(const uint8_t * ptr_, int offset_, int * newOffset = NULL)
	{
		uint16_t _res = 0;
		for (int i = 0; i < sizeof(_res); ++i)
		{
			_res += ptr_[i+offset_] << (i*8);
		}
		if(newOffset != NULL) *newOffset = offset_ +  sizeof(_res);
		return _res;
	}

	static int16_t Converter_toInt16(const uint8_t * ptr_, int offset_, int * newOffset = NULL)
	{
		int16_t _res = 0;
		for (int i = 0; i < sizeof(_res); ++i)
		{
			_res += ptr_[i+offset_] << (i*8);
		}
		if(newOffset != NULL) *newOffset = offset_ +  sizeof(_res);
		return _res;
	}

	static uint32_t Converter_toUint32(const uint8_t * ptr_, int offset_, int * newOffset = NULL)
	{
		uint32_t _res = 0;
		for (int i = 0; i < sizeof(_res); ++i)
		{
			_res += ptr_[i+offset_] << (i*8);
		}
		if(newOffset != NULL) *newOffset = offset_ +  sizeof(_res);
		return _res;
	}

	static int32_t Converter_toInt32(const uint8_t * ptr_, int offset_, int * newOffset = NULL)
	{
		int32_t _res = 0;
		for (int i = 0; i < sizeof(_res); ++i)
		{
			_res += ptr_[i+offset_] << (i*8);
		}
		if(newOffset != NULL) *newOffset = offset_ +  sizeof(_res);
		return _res;
	}

	static uint64_t Converter_toUint64(const uint8_t * ptr_, int offset_, int * newOffset = NULL)
	{
		uint64_t _res = 0;
		for (int i = 0; i < sizeof(_res); ++i)
		{
			_res += (uint64_t)ptr_[i+offset_] << (uint64_t)(i*8);
		}
		if(newOffset != NULL) *newOffset = offset_ +  sizeof(_res);
		return _res;
	}

	static int64_t Converter_toInt64(const uint8_t * ptr_, int offset_, int * newOffset = NULL)
	{
		int64_t _res = 0;
		for (int i = 0; i < sizeof(_res); ++i)
		{
			_res += (int64_t)ptr_[i+offset_] << (int64_t)(i*8);
		}
		if(newOffset != NULL) *newOffset = offset_ +  sizeof(_res);
		return _res;
	}


	template<typename T>
	T getline_as( std::string s )
	{
	    //std::stringstream convert(getline_as<std::string>(s));
		std::stringstream convert(s);

	    T value;
	    convert >> value;
	    return value;
	}

	template<class T>
	static int parsing_unbox(std::string s_, T * ptr_, int reservedSize_, std::string delimiter) {
		int cpt = 0;

		size_t pos = 0;
		std::string token;
		while ((pos = s_.find(delimiter)) != std::string::npos) {
		    token = s_.substr(0, pos);
		    s_.erase(0, pos + delimiter.length());
		    ptr_[cpt++] = getline_as<T>(token);
		    
		    if(cpt >= reservedSize_) 
		    	return cpt;
		}

		return cpt;
	};

	template<class T>
	static std::string toString(T a_)
	{
		std::ostringstream strs;
		strs << a_;
		return strs.str();
	}

	template<class T>
	static std::string parsing_box(T * ptr_, int nbr, std::string delimiter) {
		int cpt = 0;

		size_t pos = 0;
		std::string token = "";
		while (cpt < nbr) {
			token += toString(ptr_[cpt++]);
			token += delimiter;
		}

		return token;
	};

	static int id_generator(){
		return ++_id;
	}

	void print(std::string s_);
}

#endif // UTILS_H_