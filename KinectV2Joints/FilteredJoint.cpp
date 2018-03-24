#include "FilteredJoint.h"

FilteredJoint::~FilteredJoint() { }

void FilteredJoint::SetSmoothingParams(SMOOTHING_PARAM_TYPE paramType, float value) {

	if (paramType == SMOOTHING_PARAM_TYPE::SMOOTHING) {

		// Smoothing weight
		m_Smoothingparams.Smoothing = value;

	}
	else if (paramType == SMOOTHING_PARAM_TYPE::CORRECTION) {

		// Correction
		m_Smoothingparams.Correction = value;

	}
	else if (paramType == SMOOTHING_PARAM_TYPE::JITTER_RADIUS) {

		// Jitter radius (max)
		m_Smoothingparams.JitterRadius = value;

	}
	else if (paramType == SMOOTHING_PARAM_TYPE::PREDICTION) {

		// Prediction (trend)
		m_Smoothingparams.Prediction = value;

	}
	else if (paramType == SMOOTHING_PARAM_TYPE::MAX_DEVIATION_RADIUS) {

		// Max deviation radius
		m_Smoothingparams.MaxDeviationRadius = value;

	}

}

// Get the smoothing params specified by paramType
float FilteredJoint::GetSmoothingParam(SMOOTHING_PARAM_TYPE paramType) {
	
	float retVal{ -100.0f };

	if (paramType == SMOOTHING_PARAM_TYPE::SMOOTHING) {

		// Smoothing weight
		retVal = m_Smoothingparams.Smoothing;

	}
	else if (paramType == SMOOTHING_PARAM_TYPE::CORRECTION) {

		// Correction
		retVal = m_Smoothingparams.Correction;

	}
	else if (paramType == SMOOTHING_PARAM_TYPE::JITTER_RADIUS) {

		// Jitter radius (max)
		retVal = m_Smoothingparams.JitterRadius;

	}
	else if (paramType == SMOOTHING_PARAM_TYPE::PREDICTION) {

		// Prediction (trend)
		retVal = m_Smoothingparams.Prediction;

	}
	else if (paramType == SMOOTHING_PARAM_TYPE::MAX_DEVIATION_RADIUS) {

		// Max deviation radius
		retVal = m_Smoothingparams.MaxDeviationRadius;

	}

	// return the value (if no conditions above are met, the initial value of -100.0f is given)
	return retVal;

}
