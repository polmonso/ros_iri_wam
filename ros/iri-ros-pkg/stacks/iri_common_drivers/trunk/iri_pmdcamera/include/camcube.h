
#ifndef CAMCUBE_HH
#define CAMCUBE_HH

//@TODO: suppost iri exception mechanism
//@TODO: make a stand alone library

// ROS include (only needed for exceptions...  :( )
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
#include <boost/shared_ptr.hpp>
#include <pmdsdk2.h>

namespace pmd_camcube
{
  
  using namespace std;
/*  //! Macro for defining an exception with a given parent (std::runtime_error should be top parent)
  // code borrowed from drivers/laser/hokuyo_driver/hokuyo.h
#define DEF_EXCEPTION(name, parent)		\
  class name  : public parent {			\
  public:					\
    name (const char* msg) : parent (msg) {}	\
  }

  //! A standard CAMCUBE exception
    DEF_EXCEPTION (Exception, std::runtime_error);
*/
  static const unsigned int CAMCUBE_IMAGES = 3;
  static const int PMD_INTEGRATION_TIME = 600;  ///600 by default (close objects)
  
  typedef enum {camcube=0, camboard=1, invalid=100} cameraType;

  /**
  * Encapsulates the PMDcamcube drivers
  * Two cameras are supported: camcube and camboard.
  * @NOTE: camboard returns a 207x204 image but effective pixels are 200x200 that is the image size returned. Raw image is in columns, but here is returned in rows
  * @TODO: the new driver/plugins come with new features that should be added
  */

  class PmdCamcube
  {
  public:
    PmdCamcube ();
    ~PmdCamcube ();

    /**
    * Open the camera and initialises all the image buffers with the right size
    * Integration time can be set with set_integration_time()
    * @param calibration_on improves 3d point calculation
    * @param cameraType two cameras are suported, camcube and camboard
    */
    int open (bool calibration_on, cameraType type=pmd_camcube::camboard);


    int close ();

//    * @param cloud stores the point cloud with XYZ coordinates
//    *              Channel r,g,b contains rgb codded depth
//    *              Channel intensity contains the normalized intensity [0..1]
//    * @TODO: make independent from ROS: return a pointer to data, and transform to ros in the node
//    void readData (sensor_msgs::PointCloud &cloud);
    /**
    * Performs the data acquisition process
    * Images can be retrieved using get3DData(), getIntensityData()
    */
    void readData ();

    /**
    * \brief Get the different images
    *  @{
    */
    
    /**
    * Image is switched in X (coordinate system: left hand instead of right hand rule)
    * Image here is flipped before returning
    * Image is provided in column order and this function returns row order.
    * image is depth filtered using set_min_depth_limit() and set_max_depth_limit() values
    */
    float *get3DData ();
    
    float *getRaw3DData ();
    float *getIntensityData ();
    float *getAmplitudeData ();
    float *getDepthData ();
    /** @} */ // end of group1
    
//     std::string device_id_;
//     std::string lib_version_;
    /** 
    * sets the limits for the depth filtering
    * @param lim depth in m 
    */
    void set_min_depth_limit (double lim);
    void set_max_depth_limit (double lim);

    int get_width ();
    int get_height ();

    /**
    * The integration time determines the distance of best acquisition of the camera
    * Can be changed with dynamic_reconfiguration via: rosrun dynamic_reconfigure reconfigure_gui
    */
    void set_integration_time (unsigned int integration_time);
    int get_integration_time ();
    
    /**
    * apparently camboard only support 20000000Hz
    * check for the closes possible value and applies it
    * @param modulation_frequency in Hz, by default 20000000
    */
    void set_modulation_frequency (unsigned int modulation_frequency);
  private:
    // device identifier
    PMDHandle CC_hnd_;
    PMDDataDescription dd_;
    cameraType cameraType_;
    
    /**
    * \brief the different images
    * As some dead points can exist we need two images for each one, one to get from camera and one to send
    *  @{
    */
	//ImgEntry* imgEntryArray_;
    float *distance_;           ///* distance image */
//    boost::shared_ptr<float> distance_;           ///* distance image */
    float *amplitude_;          ///* amplitude image */
    float *intensity_;          ///* intensity image */
    float *coord_3D_;           ///* 3D coordinate image */
    
    float *raw_distance_;           ///* distance image */
    float *raw_amplitude_;          ///* amplitude image */
    float *raw_intensity_;          ///* intensity image */
    float *raw_coord_3D_;           ///* 3D coordinate image */
    /** @} */
    
    double min_depth_limit_;
    double max_depth_limit_;

    int width_;
    int height_;
    unsigned int integration_time_;     //En realitat no fa falta guardar el valor a la classe...
    //, modulation_freq_;
    bool opened;
/*    
    int setAutoExposure (bool on);
    int setIntegrationTime (int time);
    int getIntegrationTime ();
    int setModulationFrequency (int freq);
    int getModulationFrequency ();
    int setAmplitudeThreshold (int thresh);
    int getAmplitudeThreshold ();
*/

    void SafeCleanup ();

    /**
    * Image is given in columns, but seems to be more than that as the origin seems to be left-down and not left-up
    * Rationale: rgb camera and intensity should be oriented the same!
    * This helper function makes the mapping
    * @return adress of the corresponding i,j
    */
    inline int flipPixelPosition(int i, int j) {return ((height_-j-1)*(width_)+(width_-i-1));}
    
    /**
    * given a HSI value returns the corresponding RGB
    * @param H Hue [0..1]
    * @param S Saturation [0..1]
    * @param I Intensity [0..1]
    * TODO: maybe here is not the right place??? something like libUtils....
    */
    void hsi2rgb (float H, float S, float I, float *R, float *G, float *B);
  };
};

#endif
