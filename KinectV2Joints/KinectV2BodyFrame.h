#pragma once

#include "Kinectafx.h"
#include "essentials.h"
#include <map>

class KinectV2BodyFrame {

	private:

		Vec3	m_SensorPosition{ Vec3(0.0f) };
		Vec4	m_FloorPlane;
		float	m_BodyConfidence{ 0.0f };

		std::map<JointType, Joint> m_Joints;
		std::map<JointType, JointOrientation> m_JointOrientations;

		template <class mapType>
		void	CopyMap(mapType& src, mapType& dest) {	dest = src; };

	public:

		KinectV2BodyFrame();
		KinectV2BodyFrame(std::map<JointType, Joint>& srcJoints) { Pack(srcJoints); }
		KinectV2BodyFrame(std::map<JointType, Joint>& srcJoints, std::map<JointType, JointOrientation>& srcJointOrientations) { Pack(srcJoints, srcJointOrientations); }
		KinectV2BodyFrame(std::map<JointType, Joint>& srcJoints, std::map<JointType, JointOrientation>& srcJointOrientations, float confidence) { Pack(srcJoints, srcJointOrientations, confidence); }
		KinectV2BodyFrame(std::map<JointType, Joint>& srcJoints, std::map<JointType, JointOrientation>& srcJointOrientations, Vec3& V3SensorPos, Vec4& v4FloorPlane) { Pack(srcJoints, srcJointOrientations, V3SensorPos, v4FloorPlane); }
		KinectV2BodyFrame(std::map<JointType, Joint>& srcJoints, std::map<JointType, JointOrientation>& srcJointOrientations, float& fConfidence, Vec3& v3SensorPos, Vec4& v4Floorplane) { Pack(srcJoints, srcJointOrientations, fConfidence , v3SensorPos, v4Floorplane); }
		~KinectV2BodyFrame();

		// Copy contents to target objects. Group Accessor
		void Unpack(std::map<JointType, Joint>& destJoints) { CopyMap(m_Joints, destJoints); }
		void Unpack(std::map<JointType, JointOrientation>& destJointOrientations) { CopyMap(m_JointOrientations, destJointOrientations); }
		void Unpack(std::map<JointType, Joint>& destJoints, std::map<JointType, JointOrientation>& destJointOrientations);
		void Unpack(std::map<JointType, Joint>& destJoints, std::map<JointType, JointOrientation>& destJointOrientations, Vec3& destSensorPos, Vec4& destFloorplane);
		void Unpack(std::map<JointType, Joint>& destJoints, std::map<JointType, JointOrientation>& destJointOrientations, float& destConfidence, Vec3& destSensorPos, Vec4& destFloorplane);

		// Copy source objects to internal. Group mutator
		void Pack(std::map<JointType, Joint>& srcJoints) { CopyMap(srcJoints, m_Joints); }
		void Pack(std::map<JointType, JointOrientation>& srcJointOrientations) { CopyMap(srcJointOrientations, m_JointOrientations); }
		void Pack(std::map<JointType, Joint>& srcJoints, std::map<JointType, JointOrientation>& srcJointOrientations);
		void Pack(std::map<JointType, Joint>& srcJoints, std::map<JointType, JointOrientation>& srcJointOrientations, float confidence);
		void Pack(std::map<JointType, Joint>& srcJoints, std::map<JointType, JointOrientation>& srcJointOrientations, Vec3 sensorPos, Vec4 floorPlane);
		void Pack(std::map<JointType, Joint>& srcJoints, std::map<JointType, JointOrientation>& srcJointOrientations, float confidence, Vec3 sensorPos, Vec4 floorPlane);

		// Accessors
		float	GetConfidence() { return m_BodyConfidence; }
		Vec3	GetSensorPos() { return m_SensorPosition; }
		Vec4	GetFloorPlane() { return m_FloorPlane; }
		Joint	GetJoint(JointType type) { return m_Joints.at(type); }
		JointOrientation GetJointOrientation(JointType type) { return m_JointOrientations.at(type); }

		// Mutators
		void	SetConfidence(float conf) { m_BodyConfidence = conf; }
		void	SetSensorPos(Vec3 pos) { m_SensorPosition = pos; }
		void	SetFloorPlane(Vec4 plane) { m_FloorPlane = plane; }
		void	SetJoint(JointType type, Joint& value) { m_Joints.at(type) = value; }
		void	SetJointOrientation(JointType type, JointOrientation value) { m_JointOrientations.at(type) = value; }
		bool	isValid() { return (m_Joints.size() != 0 && m_JointOrientations.size() != 0); }

};
