#include "KinectV2JointReader.h"
#include "JointWeights.h"
#include "stdafx.h"
#include <numeric>
#include <string>

void KinectV2JointReader::OnBeforeProcess(KinectV2BodyFrame & frameIn, KinectV2BodyFrame & frameOut) { }

void KinectV2JointReader::OnAfterProcess(KinectV2BodyFrame & frameIn, KinectV2BodyFrame & frameOut) { }

void KinectV2JointReader::MirrorSkeleton_Y(std::map<JointType, Joint>& dataMap) { }	// Unimplemented

void KinectV2JointReader::MirrorSkeleton_X(std::map<JointType, Joint>& dataMap) { }	// Unimplemented

void KinectV2JointReader::MirrorSkeleton_Z(std::map<JointType, Joint>& dataMap) {

	// Cycle the map, inverting the X values
	for (auto currentJoint : dataMap) {

		dataMap[currentJoint.first].Position.Z *= -1.0f;

	}

}

// CTOR
KinectV2JointReader::KinectV2JointReader() : m_ApplyGrounding(true) {

	// Init members/data
	Reset();

	// Passthrough
	SetJointReadingMode(SkeletonJointSmoothingType::Passthrough);

}

// CTOR
KinectV2JointReader::KinectV2JointReader(SkeletonJointSmoothingType smType) : m_ApplyGrounding(true) {
	
	// Init members/data
	Reset();

	// Set the joint reading mode
	SetJointReadingMode(smType);

}

// DTOR
KinectV2JointReader::~KinectV2JointReader() {

	// Reset
	Reset();

	// Delete all members
	for (auto member : m_Joints) {

		delete member.second;

	}

	// empty map
	m_Joints.clear();

}

// Update
void KinectV2JointReader::Update(float dTime) {

	// If no new data is available, quit out (no processing necessary)
	if (!Kinect2API::HasFreshBodyData()) {

		return;
	
	}

	// Read data (if available)
	GetSensorData(m_RawFrame);

	// Pre-process event
	OnBeforeProcess(m_RawFrame, m_ProcessedFrame);

	// use 'scratch' maps for processing
	std::map<JointType, Joint> rawJoints;
	std::map<JointType, Joint> processedJoints;
	std::map<JointType, JointOrientation> rawOrientations;
	std::map<JointType, JointOrientation> processedOrientations;

	// Unpack to 'src' maps to work on
	m_RawFrame.Unpack(rawJoints, rawOrientations);

	// reflect the skeleton in Z
	MirrorSkeleton_Z(rawJoints);

	// (if enabled) 'Ground' the skeleton data (src) to the floor
	if (IsGroundingEnabled()) {

		GroundSkeleton(rawJoints, JointType_KneeLeft, JointType_KneeRight);

	}

	// Perform processing
	Process(rawJoints, rawOrientations, processedJoints, processedOrientations);

	// Post-process event
	OnAfterProcess(m_RawFrame, m_ProcessedFrame);

	// pack back into destination frame
	m_ProcessedFrame.Pack(processedJoints, processedOrientations);

}

// Set a param for smoothing
void KinectV2JointReader::SetSmoothingParam(SMOOTHING_PARAM_TYPE type, float value) {

	if (type == SMOOTHING_PARAM_TYPE::SMOOTHING) {

		// Set smoothing value
		m_SmoothingParams.Smoothing = value;

	}
	else if (type == SMOOTHING_PARAM_TYPE::CORRECTION) {

		// Set correction value
		m_SmoothingParams.Correction = value;

	}
	else if (type == SMOOTHING_PARAM_TYPE::PREDICTION) {

		// Set prediction
		m_SmoothingParams.Prediction = value;

	}
	else if (type == SMOOTHING_PARAM_TYPE::JITTER_RADIUS) {

		// Set Jitter radius
		m_SmoothingParams.JitterRadius = value;

	}
	else if (type == SMOOTHING_PARAM_TYPE::MAX_DEVIATION_RADIUS) {

		// Set max deviation
		m_SmoothingParams.MaxDeviationRadius = value;

	} // No 'else' necessary

	// Propogate update to joints
	UpdateJointSmoothingData(type, value);

}

// Get a smoothing param
float KinectV2JointReader::GetSmoothingParam(SMOOTHING_PARAM_TYPE type) {

	float fReturnValue{ -100.0f };

	if (type == SMOOTHING_PARAM_TYPE::SMOOTHING) {

		fReturnValue = GetSmoothingParams().Smoothing;

	}
	else if (type == SMOOTHING_PARAM_TYPE::CORRECTION) {

		fReturnValue = GetSmoothingParams().Correction;

	}
	else if (type == SMOOTHING_PARAM_TYPE::JITTER_RADIUS) {

		fReturnValue = GetSmoothingParams().JitterRadius;

	}
	else if (type == SMOOTHING_PARAM_TYPE::PREDICTION) {

		fReturnValue = GetSmoothingParams().Prediction;

	}
	else if (type == SMOOTHING_PARAM_TYPE::MAX_DEVIATION_RADIUS) {

		fReturnValue = GetSmoothingParams().MaxDeviationRadius;

	}

	return fReturnValue;

}

void KinectV2JointReader::SetAverageWeightingFactor(float value) {

	// Limit to 99%
	m_AverageWeightingFactor = (value > 0.999f) ? 0.99f : value;

	// If the joints are DoubleExponent Joints, set their values
	if (m_JointSmType == SkeletonJointSmoothingType::Average) {

		// Cycle joints, update
		for (auto joint : m_Joints) {

			joint.second->SetAverageWeightingFactor(value);

		}

		// done

	}
	
}

// Resize history
void KinectV2JointReader::SetGroundingHistorySize(int size, std::list<float>& srcList) {

	// store the history size
	m_GroundingHistorySize = size;

	// if the list exceeds history size, resize cut it down (if smaller, it will grow)
	if (srcList.size() > m_GroundingHistorySize) {

		// Resize (removing entries at the back; older values)
		srcList.resize(size);

	}

}

// Set the joint reading mode
void KinectV2JointReader::SetJointReadingMode(SkeletonJointSmoothingType smType) {

	// Set local state variable
	m_JointSmType = smType;

	// For 'swap' to copy previous frame of data
	FilteredJoint* newJoint{ nullptr };

	// Build map of filtered joints of type according to smType
	if (smType == SkeletonJointSmoothingType::Passthrough) {

		// Create joints
		for (auto member : KinectJoints) {

			// Current ('old') joint
			FilteredJoint* old = m_Joints[member];

			// Create a new joint from the old joint
			FilteredJoint* newJoint{ nullptr };

			if (old != nullptr) {

				// Use previous 'old' joint for data
				newJoint = new PassthroughJoint(old);

			}
			else {
				// Create a new joint and set values
				newJoint = new PassthroughJoint();

				// Set weight
				newJoint->SetWeight(jointWeights[member]);

				// Set history size
				newJoint->SetHistorySize(GetGroundingHistorySize());

				// Set smoothing params
				newJoint->SetSmoothingParams(GetSmoothingParams());

			}

			// Delete old joint
			CleanDelete(old);

			// Replace
			m_Joints[member] = newJoint;

		}

	}
	else if (smType == SkeletonJointSmoothingType::DoubleExponent) {

		// Create joints
		for (auto member : KinectJoints) {

			// Current ('old') joint
			FilteredJoint* old = m_Joints[member];

			// Create a new joint from the old joint
			FilteredJoint* newJoint{ nullptr };

			if (old != nullptr) {

				// Use previous 'old' joint for data
				newJoint = new DoubleExponentJoint(old);

			}
			else {

				// Create a new joint and set values
				newJoint = new DoubleExponentJoint();

				// Set weight
				newJoint->SetWeight(jointWeights[member]);

				// Set history size
				newJoint->SetHistorySize(GetGroundingHistorySize());

				// Set smoothing params
				newJoint->SetSmoothingParams(GetSmoothingParams());

			}


			// Delete old joint
			CleanDelete(old);

			// Replace
			m_Joints[member] = newJoint;


		}

	}
	else if (smType == SkeletonJointSmoothingType::Average) {

		// Create joints
		for (auto member : KinectJoints) {

			// Current ('old') joint
			FilteredJoint* old = m_Joints[member];

			// Create a new joint from the old joint
			FilteredJoint* newJoint{ nullptr };

			if (old != nullptr) {

				// Use previous 'old' joint for data
				newJoint = new AveragedJoint(old);

			}
			else {

				// Create a new joint and set values
				newJoint = new AveragedJoint();

				// Set weight
				newJoint->SetWeight(jointWeights[member]);

				// Set history size
				newJoint->SetHistorySize(GetGroundingHistorySize());

				// Set smoothing params
				newJoint->SetSmoothingParams(GetSmoothingParams());

			}

			// Delete old joint
			CleanDelete(old);

			// Replace
			m_Joints[member] = newJoint;


		}

	}

}

// Reset
void KinectV2JointReader::Reset() {

	// Reset the filtered joints
	for (auto member : m_Joints) {

		member.second->Reset();

	}

}

// Processing
void KinectV2JointReader::Process(std::map<JointType, Joint>& srcMap, std::map<JointType, JointOrientation>& srcOrientations, std::map<JointType, Joint>& destMap, std::map<JointType, JointOrientation>& destOrientations) {

	// Cycle the map of joints from the API, triggering each joint to process
	for (auto members : srcMap) {

		// Add the data to the filteredJoint and process
		m_Joints[members.first]->Update(members.second);

		// Get the filtered data from the filtered joint
		destMap[members.first] = m_Joints[members.first]->GetCurrentData().FilteredPosition;// GetFilteredJoint();

	}

	// Copy over orientations
	destOrientations = srcOrientations;

}

// Get a data frame
bool KinectV2JointReader::GetSensorData(KinectV2BodyFrame & Frame) {

	// If the sensor isn't functioning, drop out
	if (!Kinect2API::StatusIsWorking()) {

		return false;

	}

	// Get if available
	if (Kinect2API::HasFreshBodyData()) {

		// Get the joint data
		Kinect2API::GetDataFrame(Frame);

	}

	return true;

}

// 'Ground' the skeleton, placing both feet (or the lower foot) on the ground plane. Assume NO jumping etc.
// Note: Adapted to be any two left/right combination joints, not necessarily feet
void KinectV2JointReader::GroundSkeleton(std::map<JointType, Joint>& dataMap, JointType leftJoint, JointType rightJoint) {

	// Ensure both joints are in the frame (unpacked to the map) before processing
	std::map<JointType, Joint>::iterator itr;

	if (dataMap.size() > 0) {

		// find both joints
		for (auto currJoint : { leftJoint, rightJoint }) {

			if (dataMap.find(leftJoint) == dataMap.end()) {

				// The joint isn't found

				// Exit
				return;

			}

		}
	
	}

	// Get Y-Position of left and right foot
	float fLeftFootHeight = dataMap[leftJoint].Position.Y;
	float fRightFootHeight = dataMap[rightJoint].Position.Y;

	// Get difference between foot heights (to choose the lower foot) - make sure it's positive with fabs(..)
	float footHeightDiff = fabs(fLeftFootHeight - fRightFootHeight);

	// storage for lower foot position
	float fLowerFootY{ 0.0f };
	
	// If the difference is beyond specified tolerance
	if (footHeightDiff > FOOTHEIGHT_EPSILON) {

		// Noticeable difference between feet heights, get the lower foot
		fLowerFootY = (fLeftFootHeight < fRightFootHeight) ? fLeftFootHeight : fRightFootHeight;

	}
	else {

		// Both feet are within the same tolerance height - get midpoint of Y values
		fLowerFootY = (fLeftFootHeight + fRightFootHeight) * 0.5f;
		
	}

	// Add this value to the FRONT of the list to calculate an average
	m_avgFootYDifference.push_front(fLowerFootY);

	int historySize = GetGroundingHistorySize();

	// Check the list doesn't exceed history size. If so, 'shrink' the list
	if (m_avgFootYDifference.size() > historySize) {

		// Resize list to history size (older values [back] are removed)
		m_avgFootYDifference.resize(historySize);

	}

	// average the values int the list
	float avgFootHeight = std::accumulate(m_avgFootYDifference.begin(), m_avgFootYDifference.end(), 0.0f) / m_avgFootYDifference.size();

	// Convert stored floor value to MILLIMETRES (otherwise the skeleton will adjust in metres; not very fine controlled)
	float floorY = GetFloorYValue() * Kinectafx::KINECT_MM_TO_METRES;

	// Cycle the data map and adjust
	for (auto member : dataMap) {

		dataMap[member.first].Position.Y += (floorY - avgFootHeight);

	}

}

// Update the smoothing details per-joint
void KinectV2JointReader::UpdateJointSmoothingData(SMOOTHING_PARAM_TYPE spType, float value) {

	// Cycle the list of joints and update the smoothing param
	for (auto joint : m_Joints) {

		FilteredJoint* currentJoint = joint.second;
		currentJoint->SetSmoothingParams(spType, value);

	}

}

// Adjust the history size of each joint
void KinectV2JointReader::AdjustJointHistorySize(int size) {
	
	// Store locally
	m_JointHistorySize = size;

	// Cycle each joint, setting
	for (auto member : m_Joints) {

		member.second->SetHistorySize(size);

	}

}
