#pragma once

#include "GLMTypes.h"

#include <cmath>
#include <string>
#include <iostream>

// Nybble reading/writing
#define HINIBBLE(b) (((b) >> 4) & 0x0F)
#define LONIBBLE(b) ((b) & 0x0F)
#define MAKEBYTE(lo, hi) (lo | hi << 4);

struct Vertex;
using Recti = glm::vec4;
const float FAR_PLANE = 50000.0f;
const std::string TEXTURES_DIR = "../Assets/Textures/";

// For float comparisons
const float FLOAT_EPSILON = 0.0000001f;
inline bool fEqual(float floatA, float floatB) { return (fabs(floatA - floatB) < FLOAT_EPSILON); }
inline bool fEqual(float floatA, float floatB, float EPSILON) { return (fabs(floatA - floatB) < EPSILON); }
inline bool fMore(float floatA, float floatB) { return (floatA > (floatB + FLOAT_EPSILON)); }
inline bool fMore(float floatA, float floatB, float EPSILON) { return (floatA > (floatB + EPSILON)); }
inline bool fLess(float floatA, float floatB) { return (floatA < (floatB - FLOAT_EPSILON)); }
inline bool fLess(float floatA, float floatB, float EPSILON) { return (floatA < (floatB - EPSILON)); }

// Enumerate sensor types
enum class SENSORMATERIALTYPE {

	LEAP_LEFT = 1000,
	LEAP_RIGHT = 1001,
	KINECTV2 = 1002,
	OVR = 1003

};

template<class obj>
inline void CleanDelete(obj *& objectToDelete) {

	if (objectToDelete != nullptr) {

		delete objectToDelete;
		objectToDelete = nullptr;

	}

}

// Safe release COM
template<class Interface>
inline void CleanRelease(Interface *& pInterfaceToRelease) {

	if (pInterfaceToRelease != nullptr) {

		pInterfaceToRelease->Release();
		pInterfaceToRelease = nullptr;

	}

}