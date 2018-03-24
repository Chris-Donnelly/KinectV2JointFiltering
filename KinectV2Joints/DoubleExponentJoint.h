#pragma once

#include "FilteredJoint.h"

class DoubleExponentJoint : public FilteredJoint{

	public:

		DoubleExponentJoint() {};
		DoubleExponentJoint(FilteredJoint* src);
		~DoubleExponentJoint() { };

		inline virtual void Init() { Reset(); }
		virtual void		Reset() override;
		virtual void		Update(Joint srcJoint) override;
		virtual void		Process(Joint srcJoint, SkeletonSmoothingParams params) override;

	private: 

		long int			m_FrameCount{ 0 };
		std::list<Joint>	m_JointHistory;

		inline long int		GetFrameCounter() { return m_FrameCount; }
		inline void			SetFrameCounter(long int value) { m_FrameCount = value; }
		inline void			IncFrameCounter() { m_FrameCount++; }
		inline float		MultiplyLimit(float value, float multiplier, float limit) { return (value * multiplier > limit) ? limit : multiplier * value; }

};
