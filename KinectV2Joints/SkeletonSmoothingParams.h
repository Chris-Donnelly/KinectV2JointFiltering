#pragma once

const float SP_DEFAULT_SMOOTHING = 0.75f;
const float SP_DEFAULT_CORRECTION = 0.25f;
const float SP_DEFAULT_PREDICTION = 0.25f;
const float SP_DEFAULT_JITTERRADIUS = 0.15f;
const float SP_DEFAULT_MAXDEVRADIUS = 0.015f;
const float	DEFAULT_AVG_WEIGHT_FACTOR = 0.65f;

enum class SMOOTHING_PARAM_TYPE { SMOOTHING, CORRECTION, PREDICTION, JITTER_RADIUS, MAX_DEVIATION_RADIUS };

class SkeletonSmoothingParams {

	public:

		SkeletonSmoothingParams() : Smoothing(SP_DEFAULT_SMOOTHING), Correction(SP_DEFAULT_CORRECTION), Prediction(SP_DEFAULT_PREDICTION), JitterRadius(SP_DEFAULT_JITTERRADIUS), MaxDeviationRadius(SP_DEFAULT_MAXDEVRADIUS) {};	// default zero
		SkeletonSmoothingParams(float smooth, float corr, float pred, float jitter, float maxdev) : Smoothing(smooth), Correction(corr), Prediction(pred), JitterRadius(jitter), MaxDeviationRadius(maxdev) {
		
			// Ensures positive, avoids division by zero (adds 1mm if necessary)
			JitterRadius = fabs(JitterRadius) + ((JitterRadius == 0.0f)? 0.0001f : 0.0f);

		};

		float Smoothing{ 0.0f };
		float Correction{ 0.0f };
		float Prediction{ 0.0f };
		float JitterRadius{ 0.0f };
		float MaxDeviationRadius{ 0.0f };

};

