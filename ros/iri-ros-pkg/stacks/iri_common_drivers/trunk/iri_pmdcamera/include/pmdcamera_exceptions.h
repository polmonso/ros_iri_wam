#ifndef _PMDCAMERA_EXCEPTIONS
#define _PMDCAMERA_EXCEPTIONS

#include "exceptions.h"

/**
 * \brief Internal pmdcamera exception
 *
 */
class CPmdCameraInternalException : public CException
{
  public:
    /**
     * \brief Class constructor 
     *
     * This constructor generates a standard error message with the error code
     * provided as a parameter. The standard error message is "Internal pmdaccess 
     * error: error code   ", with the error code appended at the end. The base
     * class constructor is called, so the same prefix is appended at the 
     * beginning to label the message as an exception.
     *
     * \param where a null terminated string with the information about the name
     *              of the function, the source code filename and the line where
     *              the exception was generated. This string must be generated 
     *              by the _HERE_ macro.
     *
     * \param error_code the error codereturned by one of the pmdaccess library
     *                   functions. See the pmdaccess documentation for further
     *                   details on the possible errors.
     */
    CPmdCameraInternalException(const std::string& where,int error_code);
};

/**
 * \brief Camera driver Exception
 *
 * This class is used to report errors of the camera driver which are not caused
 * by calls to the pmdaccess library functions. These errors are normally caused by
 * providing incorrect parameters to the driver functions, attempting to
 * perform invalid operations or performing a sequence of operations in the 
 * incorrect order.
 *
 * This class is only provided to help distinguish between internal firewire
 * errors and errors of the driver itself, but it does not add any additional
 * feature over the base class CPmdCameraException.
 *
 */
class CPmdCameraException : public CException
{
  public:
    /**
     * \brief Class constructor
     *
     * This constructor initializes the error message by calling the constructor
     * of the base class. In this case, "[Exception caught] - " is also 
     * pre-appended to the error message to label it as an exception.
     *
     * \param where a null terminated string with the information about the name
     *              of the function, the source code filename and the line where
     *              the exception was generated. This string must be generated 
     *              by the _HERE_ macro.
     *
     * \param error_msg a null terminated string that contains the error message.
     *                  This string may have any valid character and there is no 
     *                  limit on its length.
     */ 
    CPmdCameraException(const std::string& where,const std::string& error_msg);
};

class CPmdCameraFeatureException : public CException
{
  public:
    CPmdCameraFeatureException(const std::string& where,const std::string& error_msg,const std::string& feature);
};

#endif
