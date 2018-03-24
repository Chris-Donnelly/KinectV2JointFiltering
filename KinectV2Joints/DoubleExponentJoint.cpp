#include "DoubleExponentJoint.h"

DoubleExponentJoint::DoubleExponentJoint(FilteredJoint* src) {

	// Initialize
	Init();

	// Set weight
	SetWeight(src->GetWeight());

	// Set smoothing params
	SetSmoothingParams(src->GetSmoothingParams());

	// Store last known frames to smooth transition
	SetPrevData(src->GetPreviousData());
	SetCurrentData(src->GetCurrentData());

	// Start with last known frame from src
	Update(src->GetRawJoint());

}

// update / process
void DoubleExponentJoint::Reset() {

	// Reset current and previous data
	SetCurrentData(FilteredJointData());
	SetPrevData(FilteredJointData());

	SetFrameCounter(0);

}

// Update
void DoubleExponentJoint::Update(Joint srcJoint) {

	// Store raw joint
	SetCurrentRawJoint(srcJoint);

	// If too much movement between the current and previous data is detected, decrement the frame counter
	if (glm::distance(toVec3(srcJoint), toVec3(GetCurrentData().FilteredPosition)) > JOINT_MAX_CHANGE) {

		// Step back one stage in filtering to partially reset the data/joint
		SetCurrentFilteredJoint(srcJoint);

	}

	// Get smoothing params
	SkeletonSmoothingParams smNewParams = GetSmoothingParams();

	// For inferred joints, increasesmoothing params
	if (srcJoint.TrackingState == TrackingState_Inferred) {

		// increase jitter radius and maximum deviation (max out at 1.0f)
		smNewParams.JitterRadius = MultiplyLimit(smNewParams.JitterRadius, 1.2f, 1.0f);
		smNewParams.MaxDeviationRadius = MultiplyLimit(smNewParams.MaxDeviationRadius, 1.2f, 1.0f);

	}
	else {

		float currentWeight = GetWeight();
		// Weight params
		smNewParams.JitterRadius = MultiplyLimit(smNewParams.JitterRadius, currentWeight, 1.0f);
		smNewParams.MaxDeviationRadius = MultiplyLimit(smNewParams.MaxDeviationRadius, currentWeight, 1.0f);

	}

	// Process using smNewParams
	Process(srcJoint, smNewParams);

}

// Process
void DoubleExponentJoint::Process(Joint srcJoint, SkeletonSmoothingParams smoothingParams) {

	// Terminology for this function: CurrentData is 'Previous', as new data comes from srcJoint

	Vec3 PrevRawPosition{ Vec3(0.0f) };
	Vec3 PrevFilteredPosition{ Vec3(0.0f) };
	Vec3 PrevTrend{ Vec3(0.0f) };
	
	Vec3 RawPosition{ Vec3(0.0f) };
	Vec3 FilteredPosition{ Vec3(0.0f) };
	Vec3 Trend{ Vec3(0.0f) };

	Vec3 PredictedPosition{ Vec3(0.0f) };
	Vec3 Difference{ Vec3(0.0f) };

	float Length{ 0.0f };

	// Get the previous data in one call
	FilteredJointData PrevData = GetCurrentData();

	// Store previous data locally
	PrevFilteredPosition = toVec3(PrevData.FilteredPosition);
	PrevRawPosition = toVec3(PrevData.RawPosition);
	PrevTrend = toVec3(PrevData.Trend);

	// Current RAW position is newest joint
	RawPosition = toVec3(srcJoint);

	// Validate joint; If invalid, the framecounter is reset
	if (IsZero(srcJoint.Position)) {

		SetFrameCounter(0);

	}

	// Initial state (no data to filter)
	if (GetFrameCounter() == 0) {
		
		// No filter data - Filtered = Input
		FilteredPosition = RawPosition;

		// Set current trend as 0
		Trend = Vec3(0.0f);

		// Increment Frame count (current)
		IncFrameCounter();

	}
	else if (GetFrameCounter() == 1) {

		// Mid point between old and new raw positions
		FilteredPosition = (RawPosition + PrevRawPosition) * 0.5f;

		// Calculate trend from difference between filtered positions
		Difference = FilteredPosition - PrevFilteredPosition;
		Trend = (Difference * smoothingParams.Correction) + (PrevTrend * (1.0f - smoothingParams.Correction));

		// Increment frame counter
		IncFrameCounter();
		
	}
	else {

		// ---- Jitter Reduction -----
		
		// Detect Jitter
		Difference = RawPosition - PrevFilteredPosition;
		Length = glm::length(Difference);
		
		// If the length is outside of the jitter radius, the movement is a jitter
		if (Length  >= smoothingParams.JitterRadius) {

			// Filter the jitter (Scaled Raw + ScaledPrevFilteredPos)
			FilteredPosition = 
				(RawPosition * (Length / smoothingParams.JitterRadius)) +
				(PrevFilteredPosition * (1.0f - Length / smoothingParams.JitterRadius));

		}
		else {
			
			// The data is a movement (not a jitter), and should remain the raw value
			FilteredPosition = RawPosition;

		}

		// ---- Double Exponent Filter ----

		// S(t) = a*x(t) + (1-a)*(S(t-1)+b(t-1))     0<=a<=1
		FilteredPosition = 
			(FilteredPosition * (1.0f - smoothingParams.Smoothing)) +
			((FilteredPosition + PrevTrend) * smoothingParams.Smoothing);

		// Update trend using Difference
		Difference = FilteredPosition - PrevFilteredPosition;
		Trend =
			(Difference * smoothingParams.Correction) +
			(PrevTrend * (1.0f - smoothingParams.Correction));

	}

	// Calculate predicted position to check latency
	PredictedPosition = FilteredPosition + (Trend * smoothingParams.Prediction);
	
	// Is the prediction too far from the raw data?
	Difference = PredictedPosition - RawPosition;
	Length = glm::length(PredictedPosition);

	if (Length > smoothingParams.MaxDeviationRadius) {

		// Adjust
		PredictedPosition =
			(PredictedPosition * (smoothingParams.MaxDeviationRadius / Length)) +
			(RawPosition * (1.0f * smoothingParams.MaxDeviationRadius / Length));

	}

	// Save data

	// Create joint from srcJoint for filtered position
	Joint newFilterJoint = srcJoint;
	newFilterJoint.Position = toPosition(FilteredPosition);

	// Create trend Joint from srcJoint
	Joint newTrendJoint = srcJoint;
	newTrendJoint.Position = toPosition(Trend);

	// Create Filtered Data and set
	FilteredJointData data = FilteredJointData(srcJoint, newFilterJoint, newTrendJoint);

	// Store the current data
	SetCurrentData(data);

}
