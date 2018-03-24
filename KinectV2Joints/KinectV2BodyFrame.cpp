#include "KinectV2BodyFrame.h"

KinectV2BodyFrame::KinectV2BodyFrame() { }

KinectV2BodyFrame::~KinectV2BodyFrame() { }

// Unpack maps from class to destination
void KinectV2BodyFrame::Unpack(std::map<JointType, Joint>& destJoints, std::map<JointType, JointOrientation>& destJointOrientations) {

	// Unpack joints
	Unpack(destJoints);

	// Unpack orientations
	Unpack(destJointOrientations);

}

// Unpack - Override to include floor sensor position & plane
void KinectV2BodyFrame::Unpack(std::map<JointType, Joint>& destJoints, std::map<JointType, JointOrientation>& destJointOrientations, Vec3 & destSensorPos, Vec4 & destFloorplane) {

	// Unpack joints, orientations
	Unpack(destJoints, destJointOrientations);

	// Get sensor position
	destSensorPos = GetSensorPos();

	// Get floor plane
	destFloorplane = GetFloorPlane();

}

// Unpack - Override to include confidence, sensor position and floor plane
void KinectV2BodyFrame::Unpack(std::map<JointType, Joint>& destJoints, std::map<JointType, JointOrientation>& destJointOrientations, float & destConfidence, Vec3 & destSensorPos, Vec4 & destFloorplane) {

	// Unpack
	Unpack(destJoints, destJointOrientations, destSensorPos, destFloorplane);

	// Get confidence
	destConfidence = GetConfidence();

}

// Pack - override for 2 maps
void KinectV2BodyFrame::Pack(std::map<JointType, Joint>& srcJoints, std::map<JointType, JointOrientation>& srcJointOrientations) {

	// Copy joints
	Pack(srcJoints);

	// copy joint orientations
	Pack(srcJointOrientations);

}

// Pack - Override to include confidence only
void KinectV2BodyFrame::Pack(std::map<JointType, Joint>& srcJoints, std::map<JointType, JointOrientation>& srcJointOrientations, float confidence) {

	// Pack joints
	Pack(srcJoints, srcJointOrientations);

	// Set confidence
	SetConfidence(confidence);

}

// Pack override, 2 maps, 2 vectors
void KinectV2BodyFrame::Pack(std::map<JointType, Joint>& srcJoints, std::map<JointType, JointOrientation>& srcJointOrientations, Vec3 sensorPos, Vec4 floorPlane) {

	// Pack two maps
	Pack(srcJoints, srcJointOrientations);

	// Set pos
	SetSensorPos(sensorPos);

	// Set floor plane
	SetFloorPlane(floorPlane);

}

// Pack all values
void KinectV2BodyFrame::Pack(std::map<JointType, Joint>& srcJoints, std::map<JointType, JointOrientation>& srcJointOrientations, float confidence, Vec3 sensorPos, Vec4 floorPlane) {

	// Pack joints & orientations, set confidene
	Pack(srcJoints, srcJointOrientations, sensorPos, floorPlane);

	// Set floor plane
	SetConfidence(confidence);

}

