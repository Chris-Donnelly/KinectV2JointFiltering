#pragma once

#include <Kinect.h>
#include "GLMTypes.h"

using KinectTrackingState = TrackingState;

namespace Kinectafx {

	const int IRFrame_W = 512;
	const int IRFrame_H = 424;
	const int IRFRAME_TOTALSIZE = IRFrame_H * IRFrame_W;
	const int SIZE_RGBA = 4;

	const float KINECT_SENSOR_DEPTH = 4500.0f;		// Maximum preferred depth for Kinect V2
	const float KINECT_METRES_TO_MM = 1000.0f;		// Metres to MM conversion multiplier
	const float KINECT_MM_TO_METRES = 0.001f;		// MM to Metres conversion multiplier

};


// Dirty quick convert Kinect joint data to glm::vec3
inline Vec3 toVec3(CameraSpacePoint p) { return Vec3(p.X, p.Y, p.Z); }
inline Vec3 toVec3(Joint j) { return Vec3(j.Position.X, j.Position.Y, j.Position.Z); }
inline Vec4 toVec4(Vector4 v) { return Vec4(v.x, v.y, v.z, v.w); }
inline Quat toQuat(Vector4 v) { return Quat(v.w, v.x, v.y, v.z); }
inline Quat toQuat(JointOrientation j) { return toQuat(j.Orientation); }

// Convert vec to joint (technically camperaspaceposition, within joint)
inline CameraSpacePoint toPosition(Vec3 vec) {
	
	CameraSpacePoint csp;
	csp.X = vec.x;
	csp.Y = vec.y;
	csp.Z = vec.z;
	return csp;

}

// Kinect Joint smoothing types
enum class SkeletonJointSmoothingType {

	Passthrough = 0,
	DoubleExponent = 1,
	Average = 2

};
