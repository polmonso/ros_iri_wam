// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _pmdcamera_driver_node_h_
#define _pmdcamera_driver_node_h_

#include <iri_base_driver/iri_base_driver_node.h>
#include "pmdcamera_driver.h"

#include <ros/ros.h>
#include <ros/package.h>
// [publisher subscriber headers]
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
// [service client headers]

// [action server client headers]

typedef union {
    struct /*anonymous*/
    {
      unsigned char Blue;
      unsigned char Green;
      unsigned char Red;
      unsigned char Alpha;
    };
    float float_value;
    long long_value;
} RGBValue;

/**
 * \brief IRI ROS Specific Driver Class
 *
 * This class inherits from the IRI Core class IriBaseNodeDriver<IriBaseDriver>, 
 * to provide an execution thread to the driver object. A complete framework  
 * with utilites to test the node functionallity or to add diagnostics to  
 * specific situations is also given. The inherit template design form allows  
 * complete access to any IriBaseDriver object implementation.
 *
 * As mentioned, tests in the different driver states can be performed through 
 * class methods such as addNodeOpenedTests() or addNodeRunningTests(). Tests
 * common to all nodes may be also executed in the pattern class IriBaseNodeDriver.
 * Similarly to the tests, diagnostics can easyly be added. See ROS Wiki for
 * more details:
 * http://www.ros.org/wiki/diagnostics/ (Tutorials: Creating a Diagnostic Analyzer)
 * http://www.ros.org/wiki/self_test/ (Example: Self Test)
 */
class PmdcameraDriverNode : public iri_base_driver::IriBaseNodeDriver<PmdcameraDriver>
{
  private:
    int width_,real_width_;
    int height_,real_height_;
    
    float max_intensity_; /**< used to normalize the intensity image to 0..255 */
    float max_amplitude_; /**< used to normalize the intensity image to 0..255 */
    float max_depth_; /**< used to normalize the intensity image to 0..255 */
    
    image_transport::ImageTransport *it_;

    /** \brief ROS publishers. */
    // [publisher attributes]
    //    image_transport::Publisher image_raw_publisher_;
    image_transport::CameraPublisher image_raw_publisher_;
    sensor_msgs::Image Image_int_msg_;
    
    image_transport::CameraPublisher image_amp_publisher_;
    sensor_msgs::Image Image_amp_msg_;
    
    image_transport::CameraPublisher image_depth_publisher_;
    sensor_msgs::Image Image_depth_msg_;
    
    ros::Publisher cloud_raw_publisher_;
    sensor_msgs::PointCloud PointCloud_msg_;
    
    ros::Publisher cloud2_raw_publisher_;
    sensor_msgs::PointCloud2 PointCloud2_msg_;
    // [subscriber attributes]

    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]
    
    /** \brief Camera info manager objects. */
//    boost::shared_ptr<CameraInfoManager>  camcube_info_manager_;
    CameraInfoManager *camcube_info_manager_;
    /** \brief Camera info data. */
    sensor_msgs::CameraInfo camcube_info_;
    
    /**
    * \brief fills the pointcloud2 with the 3D image data
    * \param coord_3D depth image as returned by driver->get3DImage
    * \param intensity intensity image as returned by driver->getIntensityImage
    */
    void assemblePointCloud2(const float* const coord_3D, const float* const intensity);

    /**
    * Fills the Image with the intensity data, normalized to 0..255
    * @todo this is putting knowledge in the node, and maybe should be in the low level driver?
    */
    void assembleIntensityImage(float* intensity);
    
    /**
    * Fills the Image with the amplitude data, normalized to 0..255
    * @todo this is putting knowledge in the node, and maybe should be in the low level driver?
    */
    void assembleAmplitudeImage(float* amplitude);
    
    /**
    * Fills the depth image with the depth data, normalized to 0..255
    * @todo this is putting knowledge in the node, and maybe should be in the low level driver?
    */
    void assembleDepthImage(float* distance);
    
    /**
    * finds the maximum intensity value 
    * used to normalize the intensity image and point values between 0..255
    */
    void findMaxIntensity(const float* intensity);
    void findMaxAmplitude(const float* amplitude);
    void findMaxDepth(const float* distance);
    
    /**
    * \brief post open hook
    * 
    * This function is called by IriBaseNodeDriver::postOpenHook(). In this function
    * specific parameters from the driver must be added so the ROS dynamic 
    * reconfigure application can update them.
    */    
    void postNodeOpenHook(void);

  public:
   /**
    * \brief constructor
    *
    * This constructor mainly creates and initializes the PmdcameraDriverNode topics
    * through the given public_node_handle object. IriBaseNodeDriver attributes 
    * may be also modified to suit node specifications.
    *
    * All kind of ROS topics (publishers, subscribers, servers or clients) can 
    * be easyly generated with the scripts in the iri_ros_scripts package. Refer
    * to ROS and IRI Wiki pages for more details:
    *
    * http://www.ros.org/wiki/ROS/Tutorials/WritingPublisherSubscriber(c++)
    * http://www.ros.org/wiki/ROS/Tutorials/WritingServiceClient(c++)
    * http://wikiri.upc.es/index.php/Robotics_Lab
    *
    * \param nh a reference to the node handle object to manage all ROS topics.
    */
    PmdcameraDriverNode(ros::NodeHandle& nh);

   /**
    * \brief Destructor
    *
    * This destructor is called when the object is about to be destroyed.
    *
    */
    ~PmdcameraDriverNode();

  protected:
   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the driver is on running state. Loop frequency 
    * can be tuned my modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects 
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
    void mainNodeThread(void);

    // [diagnostic functions]

   /**
    * \brief node add diagnostics
    *
    * In this function ROS diagnostics applied to this specific node may be
    * added. Common use diagnostics for all nodes are already called from 
    * IriBaseNodeDriver::addDiagnostics(), which also calls this function. Information
    * of how ROS diagnostics work can be readen here:
    * http://www.ros.org/wiki/diagnostics/
    * http://www.ros.org/doc/api/diagnostic_updater/html/example_8cpp-source.html
    */
    void addNodeDiagnostics(void);

    // [driver test functions]

   /**
    * \brief open status driver tests
    *
    * In this function tests checking driver's functionallity when driver_base 
    * status=open can be added. Common use tests for all nodes are already called
    * from IriBaseNodeDriver tests methods. For more details on how ROS tests work,
    * please refer to the Self Test example in:
    * http://www.ros.org/wiki/self_test/
    */
    void addNodeOpenedTests(void);

   /**
    * \brief stop status driver tests
    *
    * In this function tests checking driver's functionallity when driver_base 
    * status=stop can be added. Common use tests for all nodes are already called
    * from IriBaseNodeDriver tests methods. For more details on how ROS tests work,
    * please refer to the Self Test example in:
    * http://www.ros.org/wiki/self_test/
    */
    void addNodeStoppedTests(void);

   /**
    * \brief run status driver tests
    *
    * In this function tests checking driver's functionallity when driver_base 
    * status=run can be added. Common use tests for all nodes are already called
    * from IriBaseNodeDriver tests methods. For more details on how ROS tests work,
    * please refer to the Self Test example in:
    * http://www.ros.org/wiki/self_test/
    */
    void addNodeRunningTests(void);

   /**
    * \brief specific node dynamic reconfigure
    *
    * This function is called reconfigureHook()
    * 
    * \param level integer
    */
    void reconfigureNodeHook(int level);

};

#endif
