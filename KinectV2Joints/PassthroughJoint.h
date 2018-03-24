#pragma once
#include "FilteredJoint.h"

class PassthroughJoint : public FilteredJoint {

	public:

		PassthroughJoint() { Init(); }
		PassthroughJoint(FilteredJoint* src);
		~PassthroughJoint() {};

		inline virtual void Init() {}
		virtual void		Reset() override;
		virtual void		Update(Joint srcJoint) override;
		virtual void		Process(Joint srcJoint, SkeletonSmoothingParams params) override;

};

