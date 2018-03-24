#pragma once

#include "Kinectafx.h"

#include <map>
#include <array>

// skeleton bone for topology
class SkeletonBone {

public:

	JointType p1, p2;
	SkeletonBone() {};
	SkeletonBone(JointType a, JointType b) : p1(a), p2(b) {}

};

// Array pf available KinectV2 joints
static std::array<JointType, JOINT_COUNT> KinectJoints = {

	JointType_SpineBase,
	JointType_SpineMid,
	JointType_Neck,
	JointType_Head,
	JointType_ShoulderLeft,
	JointType_ElbowLeft,
	JointType_WristLeft,
	JointType_HandLeft,
	JointType_ShoulderRight,
	JointType_ElbowRight,
	JointType_WristRight,
	JointType_HandRight,
	JointType_HipLeft,
	JointType_KneeLeft,
	JointType_AnkleLeft,
	JointType_FootLeft,
	JointType_HipRight,
	JointType_KneeRight,
	JointType_AnkleRight,
	JointType_FootRight,
	JointType_SpineShoulder,
	JointType_HandTipLeft,
	JointType_ThumbLeft,
	JointType_HandTipRight,
	JointType_ThumbRight,

};

namespace SkeletonSettings {

	// Radius of sphere geometry which represents
	const float EntityDimensions_Joint = 10.0f;
	const float EntityDimensions_Bone = 5.0f;

	// Do not create renderable entities for any joints in this list whose value is false
	static std::map<JointType, bool> entityIsRenderable = {

		{ JointType_SpineBase,		true },
		{ JointType_SpineMid,		true },
		{ JointType_Neck,			false },
		{ JointType_Head,			false },
		{ JointType_ShoulderLeft,	true },
		{ JointType_ElbowLeft,		true },
		{ JointType_WristLeft,		true },	// Wrists are needed to attach hands. Do not change these
		{ JointType_HandLeft,		false },
		{ JointType_ShoulderRight,	true },
		{ JointType_ElbowRight,		true },
		{ JointType_WristRight,		true },	// Wrists are needed to attach hands. Do not change these
		{ JointType_HandRight,		false },
		{ JointType_HipLeft,		true },
		{ JointType_KneeLeft,		true },
		{ JointType_AnkleLeft,		true },
		{ JointType_FootLeft,		true },
		{ JointType_HipRight,		true },
		{ JointType_KneeRight,		true },
		{ JointType_AnkleRight,		true },
		{ JointType_FootRight,		true },
		{ JointType_SpineShoulder,	true },
		{ JointType_HandTipLeft,	false },
		{ JointType_ThumbLeft,		false },
		{ JointType_HandTipRight,	false },
		{ JointType_ThumbRight,		false }

	};

}

// List of joint<-->joint connections for each bone
static std::array<SkeletonBone, 22> skeletonTopology =
{
	{
		{ JointType_SpineBase,		JointType_SpineMid },
		{ JointType_SpineMid,		JointType_SpineShoulder },
		{ JointType_SpineShoulder,	JointType_ShoulderLeft },
		{ JointType_SpineShoulder,	JointType_ShoulderRight },
		{ JointType_SpineBase,		JointType_HipLeft },
		{ JointType_SpineBase,		JointType_HipRight },
		{ JointType_HipRight,		JointType_KneeRight },
		{ JointType_HipLeft,		JointType_KneeLeft },
		{ JointType_KneeLeft,		JointType_AnkleLeft },
		{ JointType_KneeRight,		JointType_AnkleRight },
		{ JointType_AnkleLeft,		JointType_FootLeft },
		{ JointType_AnkleRight,		JointType_FootRight },
		{ JointType_ShoulderLeft,	JointType_ElbowLeft },
		{ JointType_ShoulderRight,	JointType_ElbowRight },
		{ JointType_ElbowLeft,		JointType_WristLeft },
		{ JointType_ElbowRight,		JointType_WristRight },
		{ JointType_WristLeft,		JointType_HandLeft },
		{ JointType_WristLeft,		JointType_ThumbLeft },
		{ JointType_HandLeft,		JointType_HandTipLeft },
		{ JointType_WristRight,		JointType_HandRight },
		{ JointType_HandRight,		JointType_ThumbRight }

	}

};
