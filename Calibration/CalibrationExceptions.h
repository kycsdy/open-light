////////////////////////////////////////////////////////////////////////////////////////////////////
// file:	Calibration\CalibrationException.h
//
// summary:	Declares the calibration exception class
////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// system includes
#include <exception>
#include <string>

////////////////////////////////////////////////////////////////////////////////////////////////////
/// @class  CalibrationException
///
/// @brief  Exception for signalling calibration errors. 
///
/// @ingroup Calibration
///
/// @author Brett Jones
/// @date   12/12/2010
////////////////////////////////////////////////////////////////////////////////////////////////////
class CalibrationException: public std::exception
{
public:
    CalibrationException(std::string msg)
    {
        mMsg = msg;
        std::stringstream ss;
        ss << "Exception: " << msg;
        mExceptionMsg = ss.str();
    };

    virtual const char* what() const throw()
    {
        return mExceptionMsg.c_str();
    }

protected:
    std::string mMsg;

    std::string mExceptionMsg;
};

////////////////////////////////////////////////////////////////////////////////////////////////////
/// @class  HardwareNotFound
///
/// @brief  Hardware not found. 
///
/// @ingroup Calibration
///
/// @author Brett Jones
/// @date   12/12/2010
////////////////////////////////////////////////////////////////////////////////////////////////////
class HardwareNotFound: public CalibrationException
{
public:
    HardwareNotFound(std::string msg) : CalibrationException(msg)
    {
        std::stringstream ss;
        ss << mMsg << "HardwareNotFound: ";
        mMsg = ss.str();
    };
};

////////////////////////////////////////////////////////////////////////////////////////////////////
/// @class  HardwareInit
///
/// @brief  Hardware initialise. 
///
/// @ingroup Calibration
///
/// @author Brett Jones
/// @date   12/12/2010
////////////////////////////////////////////////////////////////////////////////////////////////////
class HardwareInit: public CalibrationException
{
public:
    HardwareInit(std::string msg) : CalibrationException(msg)
    {
        std::stringstream ss;
        ss << mMsg << "HardwareInit: ";
        mMsg = ss.str();
    };
};

class FileNotFound: public CalibrationException
{
public:
    FileNotFound(std::string msg) : CalibrationException(msg)
    {
        std::stringstream ss;
        ss << mMsg << "FileNotFound: ";
        mMsg = ss.str();
    };
};