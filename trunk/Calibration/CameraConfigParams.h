////////////////////////////////////////////////////////////////////////////////////////////////////
// file:	Calibration\CameraConfigParams.h
//
// summary:	Declares the camera configuration parameters class
////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>   Camera configuration parameters. </summary>
////////////////////////////////////////////////////////////////////////////////////////////////////
class CameraConfigParams
{
public:
    // Accessor functions

	virtual std::string GetExposure()				{ return mExposure; };
	virtual void SetExposure(std::string exp)		{ mExposure = exp; };

private:

	/// <summary> Camera Exposure. </summary>
	std::string mExposure;
};