
//#include <stdint.h>
#include "camcube.h"
#include <iostream>

#include "pmdcamera_exceptions.h"
//TODO: revise the exception mechanish to conform lab libs

//using namespace pmd_camcube;
using namespace std;

namespace pmd_camcube
{
////////////////////////////////////////////////////////////////////////////////
// Constructor
PmdCamcube::PmdCamcube ():CC_hnd_ (0), 
//distance_ (NULL), amplitude_ (NULL), intensity_ (NULL), coord_3D_ (NULL),
//raw_distance_ (NULL), raw_amplitude_ (NULL), raw_intensity_ (NULL), raw_coord_3D_ (NULL),
min_depth_limit_(0.0), max_depth_limit_(3.0),
width_ (200), height_ (200),
integration_time_ (PMD_INTEGRATION_TIME), opened (false)
{

}

PmdCamcube::~PmdCamcube ()
{
  close();
}


int
PmdCamcube::open (bool calibration_on, cameraType type) 
{
  int res;
  char err[256];
  switch (type) {
  case pmd_camcube::camcube:
    res = pmdOpen (&CC_hnd_, "camcube3.pap", "", "camcubeproc.ppp", "");
    break;
  case pmd_camcube::camboard:
    res = pmdOpen (&CC_hnd_, "camboard.pap", "", "camboardproc.ppp", "");
    //TODO: read this from a config file/parameter..
    pmdSourceCommand (CC_hnd_, 0, 0, "SetSoftOffset 0 -1.44");
    //does not work with camboard!!!
    //      res = pmdSourceCommand (CC_hnd_, 0, 0, "SetExposureMode DoubleExposure");
    //      res = pmdSourceCommand (CC_hnd_, 0, 0, "SetExposureMode Normal");
    //      std::cout<<"Set Exposure mode "<< res<<std::endl;
    //      char mode[16];
    //      pmdSourceCommand (CC_hnd_, mode, 16, "GetExposureMode");
    //std::cout<<"Exposure mode "<< mode<<std::endl;
    //   pmdSourceCommand (CC_hnd_, 0, 0, "SetROI 0 0 width_ height_");
    //  pmdProcesingCommand (CC_hnd_, 0, 0, "SetConsistencyCheck On");
    //  pmdProcesingCommand (CC_hnd_, 0, 0, "SetSaturationCheck On");
    break;
  default: 
    std::cout<<"Camera type not valid "<<std::endl;
    return 1;
  }
  
  cameraType_ = type;
  
  if (res != PMD_OK)
  {
    pmdGetLastError (0, err, 128);
    throw CPmdCameraException(_HERE_,err);
  }

  if (calibration_on)
    pmdSourceCommand (CC_hnd_, 0, 0, "SetLensCalibration On");

  res = pmdSetIntegrationTime (CC_hnd_, 0, integration_time_);
  
  res = pmdUpdate (CC_hnd_);
  if (res != PMD_OK)
  {
    close();
    pmdGetLastError (0, err, 128);
    throw CPmdCameraFeatureException(_HERE_,"Could transfer data", err);
  }

  res = pmdGetSourceDataDescription (CC_hnd_, &dd_);
  if (res != PMD_OK)
  {
    close();
    pmdGetLastError (0, err, 128);
    throw CPmdCameraFeatureException(_HERE_,"Could get data description", err);
  }

  if (dd_.subHeaderType != PMD_IMAGE_DATA)
  {
    close();
    pmdGetLastError (0, err, 128);
    throw CPmdCameraFeatureException(_HERE_,"Source data is not an image", err);
  }
  /*
  width_ = dd_.img.numColumns;
  height_ = dd_.img.numRows;
  */
  std::cout<<"Image starts at :"<<dd_.img.pixelOrigin<<std::endl;
  std::cout<<"freq :"<<dd_.img.modulationFrequency[0]<<" "<<dd_.img.modulationFrequency[1]<<" "<<dd_.img.modulationFrequency[2]<<" "<<dd_.img.modulationFrequency[3]<<std::endl;

  raw_distance_  = new float[dd_.img.numRows * dd_.img.numColumns];
  raw_amplitude_ = new float[dd_.img.numRows * dd_.img.numColumns];
  raw_intensity_ = new float[dd_.img.numRows * dd_.img.numColumns];
  raw_coord_3D_  = new float[dd_.img.numRows * dd_.img.numColumns * 3];

//  distance_  = new float[height_ * width_];
  distance_  = new float[height_ * width_];
  amplitude_ = new float[height_ * width_];
  intensity_ = new float[height_ * width_];
  coord_3D_  = new float[height_ * width_ * 3];
  
  printf("opening camera4 %d %d\n",width_,height_);
  printf("opening camera4 %d %d\n",dd_.img.numColumns,dd_.img.numRows);
  //cmaboard returns 204x207!!!
  
  opened = true;
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Safe Cleanup
void
PmdCamcube::SafeCleanup ()
{
std::cout<<"Cleaning all buffers"<<std::endl;
  if (opened) {
    free (distance_);
    free (amplitude_);
    free (intensity_);
    free (coord_3D_);
    free (raw_distance_);
    free (raw_amplitude_);
    free (raw_intensity_);
    free (raw_coord_3D_);
  }
  CC_hnd_ = NULL;
  distance_ = NULL;
  amplitude_ = NULL;
  intensity_ = NULL;
  coord_3D_ = NULL;
  raw_distance_ = NULL;
  raw_amplitude_ = NULL;
  raw_intensity_ = NULL;
  raw_coord_3D_ = NULL;

  opened = false;
}



int
PmdCamcube::close ()
{
  if (opened)
    if (pmdClose (CC_hnd_) != PMD_OK)
      ROS_WARN ("unable to stop pmd camera");

  // Free resources
  SafeCleanup ();
  opened = false;
  return 0;
}

float *
PmdCamcube::get3DData ()
{
  double val;
  int base,raw_base;
  
  for (int i=0;i<height_;i++){
    for (int j=0;j<width_;j++){
      
      raw_base = (i*dd_.img.numColumns+j)*3;
      
      if (cameraType_== pmd_camcube::camboard) {
        //TODO this is harcoded!!!! read a calibration file!/set_function/...
        //Rationale: rgb camera and intensity should be oriented the same!
        //so origin is different for intensity and depth images...
        base = flipPixelPosition(i,j)*3;
        // there is a rotation on the CamBoard axis
        // Origin is bottom right, so x and y should be flipped
        coord_3D_[base] = -raw_coord_3D_[raw_base+1];
        coord_3D_[base+1] = -raw_coord_3D_[raw_base];
        val = raw_coord_3D_[raw_base+2];// - 1.44;
        // coord_3D_[base] = raw_coord_3D_[raw_base+2];
        // coord_3D_[base+1] = -raw_coord_3D_[raw_base];
        // val = -raw_coord_3D_[raw_base+1];// - 1.44;
      } else {
        base = (i*width_ + width_ - j)*3;
        coord_3D_[base] = -raw_coord_3D_[raw_base];
        coord_3D_[base+1] = raw_coord_3D_[raw_base+1];
        val = raw_coord_3D_[raw_base+2];
      }
      /*
       *      if (val< min_depth_limit_) 
       *  coord_3D_[base+2] = min_depth_limit_;
       *      else if (val > max_depth_limit_)
       *  coord_3D_[base+2] = max_depth_limit_;
       *      else
       */
      coord_3D_[base+2] = val;
    }
  }
  //base = flipPixelPosition(10,100)*3;
  //int base2 = flipPixelPosition(100,100)*3;
  //int base3 = flipPixelPosition(100,190)*3;
  /*
  base = (95*dd_.img.numColumns+100)*3;
  int base2 = (100*dd_.img.numColumns+100)*3;
  int base3 = (105*dd_.img.numColumns+100)*3;
  ROS_INFO ("Man and min values...%f %f %f.. %f %f %f.. %f %f %f", 
	    raw_coord_3D_[0+base],raw_coord_3D_[1+base],raw_coord_3D_[2+base]- 1.44,
	    raw_coord_3D_[0+base2],raw_coord_3D_[1+base2],raw_coord_3D_[2+base2]- 1.44,
	    raw_coord_3D_[0+base3],raw_coord_3D_[1+base3],raw_coord_3D_[2+base3]- 1.44);
  */
 return coord_3D_;
}


float *
PmdCamcube::getRaw3DData (){
  int base,raw_base;
  for (int i=0;i<height_;i++){
    for (int j=0;j<width_;j++){
      //Funciona??? cal *3 no?
      //       raw_base = (i*(dd_.img.numColumns)+j)*3;
      raw_base = i*(dd_.img.numColumns)+j;
      //       base = flipPixelPosition(i,j)*3;
      base = flipPixelPosition(i,j);
      coord_3D_[base] = raw_coord_3D_[raw_base]; 
      //       coord_3D_[base+1] = raw_coord_3D_[raw_base+1]; 
      //       coord_3D_[base+2] = raw_coord_3D_[raw_base+2]; 
    }
  }
  return raw_coord_3D_;
}

float *
PmdCamcube::getIntensityData (){
  int base,raw_base;
  for (int i=0;i<height_;i++){
    for (int j=0;j<width_;j++){
      raw_base = i*(dd_.img.numColumns)+j;
      if (cameraType_== pmd_camcube::camboard) {
        //switch columns and rows
        base = flipPixelPosition(i,j);
      } else {
        base = i*(width_)+width_ - j;
      }
      intensity_[base] = raw_intensity_[raw_base];
    }
  }
  return intensity_;
}

float *
PmdCamcube::getAmplitudeData (){
  int base,raw_base;
  for (int i=0;i<height_;i++){
    for (int j=0;j<width_;j++){
      raw_base = i*(dd_.img.numColumns)+j;
      if (cameraType_== pmd_camcube::camboard) {
        //switch columns and rows
        base = flipPixelPosition(i,j);
      } else {
        base = i*(width_)+width_ - j;
      }
      amplitude_[base] = raw_amplitude_[raw_base];
    }
  }
  return amplitude_;
}

float *
PmdCamcube::getDepthData (){
  int base,raw_base;
  for (int i=0;i<height_;i++) {
    for (int j=0;j<width_;j++){
      raw_base = i*(dd_.img.numColumns)+j;
      if (cameraType_== pmd_camcube::camboard) {
	//switch columns and rows
	base = flipPixelPosition(i,j);
      }else{
	base = i*(width_)+width_ - j;
      }
      distance_[base] = raw_distance_[raw_base]; 
    }
  }
  return distance_;
}

// ////////////////////////////////////////////////////////////////////////////////
// // Store an image frame into the 'frame' buffer
// void PmdCamcube::readData (sensor_msgs::PointCloud& cloud) {
void
PmdCamcube::readData ()
{

  int res;
  char err[256];
  
  if (!opened)
  {
    close();
    pmdGetLastError (0, err, 128);
    throw CPmdCameraFeatureException(_HERE_,"Read attempted on NULL camera port!", err);
  }
  //double time1 = ros::Time::now ().toSec ();
  res = pmdUpdate (CC_hnd_);
  if (res != PMD_OK)
  {
    close();
    pmdGetLastError (0, err, 128);
    throw CPmdCameraFeatureException(_HERE_,"Could transfer data", err);
  }
  //double time2 = ros::Time::now ().toSec ();
  //double timestamp = (time1 + time2) / 2;

  res = pmdGetDistances (CC_hnd_, raw_distance_, dd_.img.numColumns * dd_.img.numRows * sizeof (float));
  if (res != PMD_OK)
  {
    close();
    pmdGetLastError (0, err, 128);
    throw CPmdCameraFeatureException(_HERE_,"Could not get intensity image", err);
  }

  res = pmdGet3DCoordinates (CC_hnd_, raw_coord_3D_, dd_.img.numColumns * dd_.img.numRows * sizeof (float) * 3);
  if (res != PMD_OK)
  {
    close();
    pmdGetLastError (0, err, 128);
    throw CPmdCameraFeatureException(_HERE_,"Could not get distances image", err);
  }

  res = pmdGetAmplitudes (CC_hnd_, raw_amplitude_, dd_.img.numColumns * dd_.img.numRows * sizeof (float));
  if (res != PMD_OK)
  {
    close();
    pmdGetLastError (0, err, 128);
    throw CPmdCameraFeatureException(_HERE_,"Could not get amplitude image", err);
  }

  res = pmdGetIntensities (CC_hnd_, raw_intensity_, dd_.img.numColumns * dd_.img.numRows * sizeof (float));
  if (res != PMD_OK)
  {
    close();
    pmdGetLastError (0, err, 128);
    throw CPmdCameraFeatureException(_HERE_,"Could not get intensity image", err);
  }

/*  cloud.header.stamp=ros::Time(timestamp);
  //image_distance.header.stamp=   image_intensity.header.stamp=image_amplitude.header.stamp=ros::Time(timestamp);
  
  size_t cloud_size=dd_.img.numColumns * dd_.img.numRows;
  // Fill in the cloud data
  cloud.points.resize(cloud_size);
    
  float maxIntens=0.0;
  for (unsigned int i=0;i<dd_.img.numRows;i++)
    {
      for (unsigned int j=0;j<dd_.img.numColumns;j++)
      {
	cloud.points[i*(dd_.img.numRows)+j].x= (coord_3D_[0+i*(dd_.img.numRows*3)+j*3]);
	cloud.points[i*(dd_.img.numRows)+j].y= (coord_3D_[1+i*(dd_.img.numRows*3)+j*3]);
	//satura els punts llunyans
	  cloud.points[i*(dd_.img.numRows)+j].z= (coord_3D_[2+i*(dd_.img.numRows*3)+j*3]);
	//save the corresponding intensity value
//	cloud.points[i*(dd_.img.numRows)+j].rgb= intensity_[i*(dd_.img.numRows)+j];
	//get the max for posterior normalization
	if (intensity_[i*(dd_.img.numRows)+j]>maxIntens) 
	  maxIntens=intensity_[i*(dd_.img.numRows)+j];
      }
    }

    //we need a second pass through the point cloud to normalize intensity... :(
    //Escala entre 0.15 i 0.5m
    cloud.channels.resize (4);
    cloud.channels[0].name="r";
    cloud.channels[0].values.resize (cloud_size);
    cloud.channels[1].name="g";
    cloud.channels[1].values.resize (cloud_size);
    cloud.channels[2].name="b";
    cloud.channels[2].values.resize (cloud_size);
    cloud.channels[3].name="intensity";
    cloud.channels[3].values.resize (cloud_size);
    for (int i=0;i<cloud_size;i++)
    {
//       if ((cloud_filtered.points[i].z<15)||(cloudXYZ.points[i].z>50)) {
// 	 cloud.channels[0].values[i]= 0;
// 	 cloud.channels[1].values[i]= 0;
// 	 cloud.channels[2].values[i]= 0;
//       } else {
	  //use the hsi model to get a different color depending on the depth
	  hsi2rgb((cloud.points[i].z-min_depth_limit_)/(max_depth_limit_-min_depth_limit_),1,.5,
		  &(cloud.channels[0].values[i]),
		  &(cloud.channels[1].values[i]),
		  &(cloud.channels[2].values[i]));
//       }
    //normalize the intensity image between [0..1]
      cloud.channels[3].values[i]= intensity_[i]/maxIntens;	
    }
    */
  return;
}

int
PmdCamcube::get_width ()
{
  return width_;
}

int
PmdCamcube::get_height ()
{
  return height_;
}

void 
PmdCamcube::set_min_depth_limit (double lim)
{
  min_depth_limit_ = lim;
}

void 
PmdCamcube::set_max_depth_limit (double lim)
{
  max_depth_limit_ = lim;  
}

void
PmdCamcube::set_integration_time (unsigned int integration_time)
{
  int res;  
  char err[256];
  unsigned int it;
  //ROS_WARN ("Ask integration time to.............................. %d", integration_time);
  res = pmdGetValidIntegrationTime (CC_hnd_, &it, 0, CloseTo, integration_time);
  if (res != PMD_OK)
  {
    close();
    pmdGetLastError (0, err, 128);
    throw CPmdCameraFeatureException(_HERE_,"SetIntegrationTime not valid", err);
  }
  //ROS_WARN ("Set integration time to.............................. %d", it);
  res = pmdSetIntegrationTime (CC_hnd_, 0, it);
  // res = pmdUpdate (CC_hnd_);
  if (res != PMD_OK)
  {
    close();
    pmdGetLastError (0, err, 128);
    throw CPmdCameraFeatureException(_HERE_,"SetIntegrationTime not valid", err);
  }
}

void
PmdCamcube::set_modulation_frequency (unsigned int modulation_frequency)
{
  int res;  
  char err[256];
  unsigned int it;
//  ROS_WARN ("Ask freq time to.............................. %d", modulation_frequency);
  res = pmdGetValidModulationFrequency (CC_hnd_, &it, 0, CloseTo, modulation_frequency);
  if (res != PMD_OK)
  {
    close();
    pmdGetLastError (0, err, 128);
    throw CPmdCameraFeatureException(_HERE_,"SetModulationFrequency not valid", err);
  }
//  ROS_WARN ("Set freq time to.............................. %d", it);
  res = pmdSetModulationFrequency (CC_hnd_, 0, it);
  // res = pmdUpdate (CC_hnd_);
  if (res != PMD_OK)
  {
    close();
    pmdGetLastError (0, err, 128);
    throw CPmdCameraFeatureException(_HERE_,"SetModulationFrequency not valid", err);
  }
}

int
PmdCamcube::get_integration_time ()
{
  unsigned int time;
  pmdGetIntegrationTime (CC_hnd_, &time, 0);
  return time;
}

void
PmdCamcube::hsi2rgb (float H, float S, float I, float *R, float *G, float *B)
{
  double domainOffset = 0.0;
  if (H < 1.0 / 6.0)
  {                             // red domain; green acending
    domainOffset = H;
    *R = I;
    *B = I * (1 - S);
    *G = *B + (I - *B) * domainOffset * 6;
  }
  else
  {
    if (H < 2.0 / 6)
    {                           // yellow domain; red acending
      domainOffset = H - 1.0 / 6.0;
      *G = I;
      *B = I * (1 - S);
      *R = *G - (I - *B) * domainOffset * 6;
    }
    else
    {
      if (H < 3.0 / 6)
      {                         // green domain; blue descending
        domainOffset = H - 2.0 / 6;
        *G = I;
        *R = I * (1 - S);
        *B = *R + (I - *R) * domainOffset * 6;
      }
      else
      {
        if (H < 4.0 / 6)
        {                       // cyan domain, green acsending
          domainOffset = H - 3.0 / 6;
          *B = I;
          *R = I * (1 - S);
          *G = *B - (I - *R) * domainOffset * 6;
        }
        else
        {
          if (H < 5.0 / 6)
          {                     // blue domain, red ascending
            domainOffset = H - 4.0 / 6;
            *B = I;
            *G = I * (1 - S);
            *R = *G + (I - *G) * domainOffset * 6;
          }
          else
          {                     // magenta domain, blue descending
            domainOffset = H - 5.0 / 6;
            *R = I;
            *G = I * (1 - S);
            *B = *R - (I - *G) * domainOffset * 6;
          }
        }
      }
    }
  }
}

}
