#include "pmdcamera_exceptions.h"
#include <sstream>
#include <string.h>
#include <stdio.h>

const std::string PmdCamera_error_message="Internal PmdCamera error: error code ";
const std::string feature_error_message="Feature error: ";

CPmdCameraInternalException::CPmdCameraInternalException(const std::string& where,int error_code):CException(where,PmdCamera_error_message)
{
  std::stringstream text;

  text << error_code;
  this->error_msg+=text.str();
}

CPmdCameraException::CPmdCameraException(const std::string& where,const std::string& error_msg):CException(where,error_msg)
{
  /* do nothing */
}

CPmdCameraFeatureException::CPmdCameraFeatureException(const std::string& where,const std::string& feature,const std::string& error_msg):CException(where,feature_error_message)
{
  this->error_msg+=error_msg;
  this->error_msg+=feature;
}