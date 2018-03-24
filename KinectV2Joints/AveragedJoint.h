#pragma once

#include "FilteredJoint.h"
#include <array>
#include <list>

class AveragedJoint : public FilteredJoint {

	public:

		AveragedJoint() { Init(); }
		AveragedJoint(FilteredJoint* src);
		~AveragedJoint();

		// Logic
		inline virtual void Init() {}
		virtual void		Reset() override;
		virtual void		Update(Joint srcJoint) override;
		virtual void		Process(Joint srcJoint, SkeletonSmoothingParams params) override;

	private:

		std::list<Joint> m_JointHistory;

};
