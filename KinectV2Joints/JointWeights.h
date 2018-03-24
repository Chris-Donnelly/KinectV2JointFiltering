#pragma once

#include "Kinectafx.h"
#include <map>

/*
	Weights for specific joints - value is multipled into smoothing parameter
	- values less than 1.0 give less smoothing, closer to raw data (jitters/deviations included)
	- values more than 1.0 give more smoothing, for joints which are inferred, or obsucred
	- values of 1.0 are given normal smoothing values
	Shoulders/elbows may need higher weights as they are obscurable and lead to jitter/deviation
*/
static std::map<JointType, float> jointWeights = {

	{	JointType_SpineBase, 1.0f },
	{	JointType_SpineMid, 1.0f },
	{	JointType_Neck, 1.0f },
	{	JointType_Head, 1.0f },
	{	JointType_ShoulderLeft, 1.0f },
	{	JointType_ElbowLeft, 1.0f },
	{	JointType_WristLeft, 1.0f },
	{	JointType_HandLeft, 1.0f },
	{	JointType_ShoulderRight, 1.0f },
	{	JointType_ElbowRight, 1.0f },
	{	JointType_WristRight, 1.0f },
	{	JointType_HandRight, 1.0f },
	{	JointType_HipLeft, 1.0f },
	{	JointType_KneeLeft, 1.0f },
	{	JointType_AnkleLeft, 1.0f },
	{	JointType_FootLeft, 1.0f },
	{	JointType_HipRight, 1.0f },
	{	JointType_KneeRight, 1.0f },
	{	JointType_AnkleRight, 1.0f },
	{	JointType_FootRight, 1.0f },
	{	JointType_SpineShoulder, 1.0f },
	{	JointType_HandTipLeft, 1.0f },
	{	JointType_ThumbLeft, 1.0f },
	{	JointType_HandTipRight, 1.0f },
	{	JointType_ThumbRight, 1.0f }

};

