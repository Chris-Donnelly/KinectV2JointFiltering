#pragma once

#include "Kinectafx.h"
#include "KinectV2BodyFrame.h"
#include "essentials.h"
#include "SkeletonSmoothingParams.h"

#include <map>
#include <list>

// Structure to hold filtered joint data
struct FilteredJointData {

	FilteredJointData() : RawPosition(Joint()), FilteredPosition(Joint()), Trend(Joint()), FrameCount(0) {};
	FilteredJointData(Joint vRawPos, Joint vFilterPos, Joint vTrend) : RawPosition(vRawPos), FilteredPosition(vFilterPos), Trend(vTrend), FrameCount(0) {};
	FilteredJointData(Vec3 vRawPos, Vec3 vFilterPos, Vec3 vTrend) {
		RawPosition.Position = toPosition(vRawPos);
		FilteredPosition.Position = toPosition(vFilterPos);
		Trend.Position = toPosition(vTrend);
	};
	FilteredJointData(Vec3 vRawPos, Vec3 vFilterPos, Vec3 vTrend, long int fCount) {
		RawPosition.Position = toPosition(vRawPos);
		FilteredPosition.Position = toPosition(vFilterPos);
		Trend.Position = toPosition(vTrend);
		FrameCount = fCount;
	};

	Joint	RawPosition;
	Joint	FilteredPosition;
	Joint	Trend;
	long int FrameCount;

};

enum class FilteredJointDataTime {
	Current = 1,
	Previous = 2
};

// Quick aliases
const FilteredJointDataTime TimeCurrent = FilteredJointDataTime::Current;
const FilteredJointDataTime TimePrev = FilteredJointDataTime::Previous;

const float JOINT_MAX_CHANGE = 200.0f * Kinectafx::KINECT_MM_TO_METRES; // 200mm | 20cm | 0.2m

class FilteredJoint {

	protected:

		float						m_AverageWeightingFactor{ DEFAULT_AVG_WEIGHT_FACTOR };
		float						m_Weight{ 1.0f };
		int							m_HistorySize{ 8 };
		SkeletonSmoothingParams		m_Smoothingparams;
		FilteredJointData			m_PrevData{ FilteredJointData() };
		FilteredJointData			m_CurrentData{ FilteredJointData() };

		// Utility
		inline bool		IsZero(const Vec3& v3) { return fEqual(v3.x, 0.0f) && fEqual(v3.y, 0.0f) && fEqual(v3.z, 0.0f); }
		inline bool		IsZero(const Joint& j) { return fEqual(j.Position.X, 0.0f) && fEqual(j.Position.Y, 0.0f) && fEqual(j.Position.Z, 0.0f); }
		inline bool		IsZero(const CameraSpacePoint& csp) { return fEqual(csp.X, 0.0f) && fEqual(csp.Y, 0.0f) && fEqual(csp.Z, 0.0f); }

		// mutators: Set new data (current data is set as previous data)
		inline void		SetCurrentData(FilteredJointData data) { SetPrevData(m_CurrentData); m_CurrentData = data; }
		inline void		SetCurrentFilteredJoint(Joint jFiltered) { SetPrevFilteredJoint(GetFilteredJoint()); m_CurrentData.FilteredPosition = jFiltered; }
		inline void		SetCurrentRawJoint(Joint jRaw) { SetPrevRawJoint(GetRawJoint()); m_CurrentData.RawPosition = jRaw; }
		inline void		SetCurrentTrend(Joint jTrend) { SetPrevTrend(GetTrend()); m_CurrentData.Trend = jTrend; }

		// mutators: set previous data
		inline void		SetPrevData(FilteredJointData data) { m_PrevData = data; }
		inline void		SetPrevFilteredJoint(Joint jFiltered) { m_PrevData.FilteredPosition = jFiltered; }
		inline void		SetPrevRawJoint(Joint jRaw) { m_PrevData.RawPosition = jRaw; }
		inline void		SetPrevTrend(Joint jTrend) { m_PrevData.Trend = jTrend; }

		// Accessors
		inline Joint	GetCurrentFilteredJoint() { return m_CurrentData.FilteredPosition; }
		inline Joint	GetCurrentRawJoint() { return m_CurrentData.RawPosition; }
		inline Joint	GetCurrentTrend() { return m_CurrentData.Trend; }
		inline Joint	GetPrevFilteredJoint() { return m_PrevData.FilteredPosition; }
		inline Joint	GetPrevRawJoint() { return m_PrevData.RawPosition; }
		inline Joint	GetPrevTrend() { return m_PrevData.Trend; }

	public:

		FilteredJoint() : m_Weight(1.0f) {};
		FilteredJoint(float weight) : m_Weight(weight) {}
		FilteredJoint(float weight, SkeletonSmoothingParams params) : m_Weight(weight), m_Smoothingparams(params) {}
		virtual ~FilteredJoint() = 0;

		// Logic
		virtual void	Reset() = 0;
		virtual void	Init() = 0;
		virtual void	Update(Joint srcJoint) = 0;
		virtual void	Process(Joint srcJoint, SkeletonSmoothingParams params) = 0;

		// Params
		inline void		SetSmoothingParams(SkeletonSmoothingParams params) { m_Smoothingparams = params; }
		void			SetSmoothingParams(SMOOTHING_PARAM_TYPE paramType, float value);
		float			GetSmoothingParam(SMOOTHING_PARAM_TYPE paramType);
		SkeletonSmoothingParams	GetSmoothingParams() { return m_Smoothingparams; }
		inline void		SetAverageWeightingFactor(float value) { m_AverageWeightingFactor = value; }

		// Accessors
		inline FilteredJointData GetCurrentData() { return m_CurrentData; }
		inline FilteredJointData GetPreviousData() { return m_PrevData; }
		inline Joint	GetFilteredJoint() { return GetCurrentFilteredJoint(); }
		inline Joint	GetRawJoint() { return GetCurrentRawJoint(); }
		inline Joint	GetTrend() { return GetCurrentTrend(); }
		inline float	GetAverageWeightingFactor() { return m_AverageWeightingFactor; }

		// Weight mutator and accessor
		inline void		SetWeight(float value) { m_Weight = value; }
		inline float	GetWeight() { return m_Weight; }

		inline int		GetHistorySize() { return m_HistorySize; }
		inline void		SetHistorySize(int value) { m_HistorySize = value; }

};

