#pragma once

#include "Kinect2API.h"
#include "essentials.h"
#include "KinectV2BodyFrame.h"
#include "SkeletonSmoothingParams.h"

// Filtered joint types. Base "Filteredjoint.h" included as base
#include "AveragedJoint.h"
#include "DoubleExponentJoint.h"
#include "PassthroughJoint.h"
#include "Skeleton.h"

#include <list>
#include <array>

// Epsilon foot height distance in millimetres (allowed difference in height between feet before one is judged lower than the other)
// Note: If not multiplied by KINECT_METRES_TO_MM, this will be calculated in metres (as with Kinect Data)
const float	FOOTHEIGHT_EPSILON = (15.0f * Kinectafx::KINECT_MM_TO_METRES);	// 15mm | 1.5cm | 0.015m
const int	DEFAULT_HISTORY = 10;

class KinectV2JointReader {

	protected:

		float			m_AverageWeightingFactor{ DEFAULT_AVG_WEIGHT_FACTOR };
		float			m_FloorY{ 0.0f };
		float			m_BodyConfidence{ 0.0f };
		bool			m_ApplyGrounding{ true };
		int				m_GroundingHistorySize{ DEFAULT_HISTORY };
		int				m_JointHistorySize{ DEFAULT_HISTORY };

		KinectV2BodyFrame	m_RawFrame;
		KinectV2BodyFrame	m_ProcessedFrame;

		SkeletonJointSmoothingType	m_JointSmType;
		SkeletonSmoothingParams		m_SmoothingParams;

		std::list<float>	m_avgFootYDifference;
		std::map<JointType, FilteredJoint*> m_Joints;

		virtual void	Process(std::map<JointType, Joint>& srcMap, std::map<JointType, JointOrientation>& srcOrientations, std::map<JointType, Joint>&destMap, std::map<JointType, JointOrientation>& destOrientations);
		virtual void	OnBeforeProcess(KinectV2BodyFrame& frameIn, KinectV2BodyFrame& frameOut);
		virtual void	OnAfterProcess(KinectV2BodyFrame& frameIn, KinectV2BodyFrame& frameOut);
		bool			GetSensorData(KinectV2BodyFrame& Frame);
		void			GroundSkeleton(std::map<JointType, Joint>&dataMap, JointType leftJoint, JointType rightJoint);
		void			UpdateJointSmoothingData(SMOOTHING_PARAM_TYPE spType, float value);
		void			AdjustJointHistorySize(int size);

		// Mirroring
		void	MirrorSkeleton_X(std::map<JointType, Joint>&dataMap);
		void	MirrorSkeleton_Y(std::map<JointType, Joint>&dataMap);
		void	MirrorSkeleton_Z(std::map<JointType, Joint>&dataMap);

	public:

		KinectV2JointReader();
		KinectV2JointReader(SkeletonJointSmoothingType smtype);
		virtual ~KinectV2JointReader();

		virtual void	Reset();
		virtual void	Update(float dTime);
		inline void		GetRawDataFrame(KinectV2BodyFrame& destFrame) { destFrame = m_RawFrame; }

		// Smoothing Params
		inline void		SetSmoothingParams(SkeletonSmoothingParams params) { m_SmoothingParams = params; }
		inline SkeletonSmoothingParams GetSmoothingParams() { return m_SmoothingParams; }
		void			SetSmoothingParam(SMOOTHING_PARAM_TYPE type, float value);
		float			GetSmoothingParam(SMOOTHING_PARAM_TYPE type);

		// Body Confidence
		inline float	GetBodyConfidence() { return m_BodyConfidence; }
		inline void		GetProcessedDataFrame(KinectV2BodyFrame& destFrame) { destFrame = m_ProcessedFrame; }

		// Weighting factor
		void			SetAverageWeightingFactor(float value);
		inline	float	GetAverageWeightingFactor() { return m_AverageWeightingFactor; }

		// Floor and grounding
		inline void		SetFloorYValue(float YVal) { m_FloorY = YVal; }
		inline float	GetFloorYValue() { return m_FloorY; }
		inline void		EnableGrounding(bool flag) { m_ApplyGrounding = flag; }
		inline bool		IsGroundingEnabled() { return m_ApplyGrounding; }

		// Grounding History Size
		inline void		SetGroundingHistorySize(int size) { SetGroundingHistorySize(size, m_avgFootYDifference); }
		inline int		GetGroundingHistorySize() { return m_GroundingHistorySize; }
		void			SetGroundingHistorySize(int size, std::list<float>& srcList);
		// Joint history size
		inline void		SetJointHistorySize(int size) { AdjustJointHistorySize(size); }
		inline int		GetJointHistorySize() { return m_JointHistorySize; }

		// Reading mode
		void			SetJointReadingMode(SkeletonJointSmoothingType type);
		SkeletonJointSmoothingType		GetJointReadingMode() { return m_JointSmType; }

};
