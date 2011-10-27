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

#ifndef _iri_clothes_grasping_point_alg_node_h_
#define _iri_clothes_grasping_point_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "iri_clothes_grasping_point_alg.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


// [publisher subscriber headers]
#include <iri_clothes_grasping_point/GraspingPointList.h>

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
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class IriClothesGraspingPointAlgNode : public algorithm_base::IriBaseAlgorithm<IriClothesGraspingPointAlgorithm>
{
  private:
    int binary_threshold;
    int canny_threshold;
    int canny_max_thresh; 
    void draw_contours_center(cv::Mat src_img);
    void draw_intersections(cv::Mat src_img);
    void count_labels(cv::Mat src_img, std::set<float>& labels);

    /// -1 stands for random selection
    void binaryObjectSelection(const cv::Mat labelled_image, cv::Mat& binary_image, unsigned char Red, unsigned char Green, unsigned char Blue);

    image_transport::ImageTransport imgtransport_;
    // [publisher attributes]
    ros::Publisher point_list_publisher_;
    iri_clothes_grasping_point::GraspingPointList GraspingPointList_msg_;
    image_transport::Publisher image_pub_;

    // [subscriber attributes]
    image_transport::Subscriber image_sub_;
    
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]

  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    IriClothesGraspingPointAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~IriClothesGraspingPointAlgNode(void);

  protected:
   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the algorithm is on running state. Loop frequency 
    * can be tuned by modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects 
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
    void mainNodeThread(void);

   /**
    * \brief dynamic reconfigure server callback
    * 
    * This method is called whenever a new configuration is received through
    * the dynamic reconfigure. The derivated generic algorithm class must 
    * implement it.
    *
    * \param config an object with new configuration from all algorithm 
    *               parameters defined in the config file.
    * \param level  integer referring the level in which the configuration
    *               has been changed.
    */
    void node_config_update(Config &config, uint32_t level);

   /**
    * \brief node add diagnostics
    *
    * In this abstract function additional ROS diagnostics applied to the 
    * specific algorithms may be added.
    */
    void addNodeDiagnostics(void);

    // [diagnostic functions]
    
    // [test functions]
};

#endif
