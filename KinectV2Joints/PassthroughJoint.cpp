#include "PassthroughJoint.h"

PassthroughJoint::PassthroughJoint(FilteredJoint* src) {

	// Init
	Init();

	// Set weight
	SetWeight(src->GetWeight());

	// Set smoothing params
	SetSmoothingParams(src->GetSmoothingParams());

	// Set History size
	SetHistorySize(src->GetHistorySize());

	// Start with last known frame from src
	Update(src->GetRawJoint());

}

void PassthroughJoint::Reset() { }

// Any update logic and processing
void PassthroughJoint::Update(Joint srcJoint) {

	// Store raw joint
	SetCurrentRawJoint(srcJoint);

	// simply process
	Process(srcJoint, GetSmoothingParams());

}

void PassthroughJoint::Process(Joint srcJoint, SkeletonSmoothingParams params) {

	// In passthrough the filtered and raw position are the same
	SetCurrentFilteredJoint(srcJoint);

	// Ignore trend variable for passthrough

}
