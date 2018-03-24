#include "AveragedJoint.h"

AveragedJoint::AveragedJoint(FilteredJoint* src) {

	Init();

	// Set weight
	SetWeight(src->GetWeight());

	// Set smoothing params
	SetSmoothingParams(src->GetSmoothingParams());

	// Start with last known frame from src
	Update(src->GetRawJoint());

}

AveragedJoint::~AveragedJoint() { }

// Reset
void AveragedJoint::Reset() {

	// Clear the history
	m_JointHistory.clear();

}

// Update logic
void AveragedJoint::Update(Joint srcJoint) { 

	// Store the raw joint
	SetCurrentRawJoint(srcJoint);

	// Call upon processing
	Process(srcJoint, GetSmoothingParams());

}

// Processing
void AveragedJoint::Process(Joint srcJoint, SkeletonSmoothingParams params) {

	// Add the Joint to the history list
	m_JointHistory.push_front(srcJoint);

	// Ensure the history size doesn't exceed the target history size
	int targetHistSize = GetHistorySize();
	if (m_JointHistory.size() > targetHistSize) {

		// Resize (removes older values at the 'back')
		m_JointHistory.resize(targetHistSize);

	}

	// Collect running total
	float totalX{ 0.0f }, totalY{ 0.0f }, totalZ{ 0.0f };
	float TotalWeights = 0.0f;
	float CurrentWeight = 1.0f;
	float WeightFactor = (GetAverageWeightingFactor() * GetWeight());

	for (auto member : m_JointHistory) {

		totalX += member.Position.X * CurrentWeight;
		totalY += member.Position.Y * CurrentWeight;
		totalZ += member.Position.Z * CurrentWeight;

		TotalWeights += CurrentWeight;
		CurrentWeight *= WeightFactor;

	}

	// Store in a new joint (keep properties of source joint)
	Joint filteredJoint = srcJoint;

	// Calculate divisor (reciprocal) for average (avoids extra divisions)
	//float fDivisor = 1.0f / m_JointHistory.size();
	float fDivisor = 1.0f / TotalWeights;

	// Store averaged values
	filteredJoint.Position.X = totalX *= fDivisor;
	filteredJoint.Position.Y = totalY *= fDivisor;
	filteredJoint.Position.Z = totalZ *= fDivisor;

	// In passthrough the filtered and raw position are the same
	SetCurrentFilteredJoint(filteredJoint);

}

