#include "hokuyo_laser3d_driver_node.h"


HokuyoLaser3dDriverNode::HokuyoLaser3dDriverNode(ros::NodeHandle &nh) : iri_base_driver::IriBaseNodeDriver<HokuyoLaser3dDriver>(nh)
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  this->evnts.push_back(NEW3DSCAN);       // event 0
  this->evnts.push_back(ENDACQUISITION);  // event 1

  // [init publishers]
  this->h3dcloud_publisher_ = this->public_node_handle_.advertise<sensor_msgs::PointCloud2>("h3dcloud", 100);
  this->cloud_publisher_ = this->public_node_handle_.advertise<sensor_msgs::PointCloud>("cloud", 100);

  // [init subscribers]

  // [init services]
  this->get_3d_scan_server_ = this->public_node_handle_.advertiseService("get_3d_scan", &HokuyoLaser3dDriverNode::get_3d_scanCallback, this);
  this->do_noncontinuous = false;
  // [init clients]

  // [init action servers]

  // [init action clients]
}

void HokuyoLaser3dDriverNode::mainNodeThread(void)
{
  //lock access to driver if necessary
  this->driver_.lock();

  // [fill msg Header if necessary]
  //<publisher_name>.header.stamp = ros::Time::now();
  //<publisher_name>.header.frame_id = "<publisher_topic_name>";

  //ROS_INFO("cont: %d noncont: %d",this->driver_.config_.continuous,this->do_noncontinuous);

  // [fill msg structures]
  //this->PointCloud_msg.data = my_var;
  if( this->driver_.config_.continuous )
  {
    this->event_server = CEventServer::instance();

    int the_event;
    bool end = false;

    try
    {
      clouddataraw *raw_scan;
      raw_scan = new clouddataraw;
      //xyz_scan = new clouddataxyz;
      the_event = this->event_server->wait_first(this->evnts);
      if(the_event==1)
      {

        end = true;

      }else{

        clouddataxyz xyz_scan;
        this->driver_.get3DScan(*raw_scan);
        this->driver_.cloudToXYZ(*raw_scan, xyz_scan);

        if(this->driver_.config_.publish_pointcloud)
        {
          this->createPointCloud(xyz_scan);
          // [publish messages]
          this->cloud_publisher_.publish(this->PointCloud_msg_);
          ROS_INFO("PointCloud Published");
        }

        if(this->driver_.config_.publish_pointcloud2)
        {
          this->createPointCloud2(xyz_scan,this->PointCloud2_msg_);
          // [publish messages]
          this->h3dcloud_publisher_.publish(this->PointCloud2_msg_);
          ROS_INFO("PointCloud2 Published");
        }

      }
      delete raw_scan;
    }
    catch(CException &e)
    {
      ROS_ERROR("'%s'",e.what().c_str());
    }


  }else
  {

    if(this->do_noncontinuous)
    {
      clouddataxyz xyz_scan;

      this->driver_.single3DScan(xyz_scan);

      if(this->driver_.config_.publish_pointcloud)
      {
        this->createPointCloud(xyz_scan);
        // [publish messages]
        this->cloud_publisher_.publish(this->PointCloud_msg_);
        ROS_INFO("PointCloud Published");
      }

      if(this->driver_.config_.publish_pointcloud2)
      {
        this->createPointCloud2(xyz_scan,this->PointCloud2_msg_);
        // [publish messages]
        this->h3dcloud_publisher_.publish(this->PointCloud2_msg_);
        ROS_INFO("PointCloud2 Published");
      }

      this->do_noncontinuous = false;

    }else{

    }

  }
  // [fill srv structure and make request to the server]


  // [fill action structure and make request to the action server]



  //delete xyz_scan;


  //unlock access to driver if previously blocked
  this->driver_.unlock();
}

/*  [subscriber callbacks] */

/*  [service callbacks] */
bool HokuyoLaser3dDriverNode::get_3d_scanCallback(iri_hokuyo_laser3d::Get3DScan::Request &req, iri_hokuyo_laser3d::Get3DScan::Response &res)
{
  //lock access to driver if necessary
  //this->driver_.lock();

  ROS_INFO("PointCloud Requested");

  if( !this->driver_.config_.continuous )
  {
    this->do_noncontinuous = true;
    res.success = 1;
  }else{
    res.success = 0;
  }
  //unlock driver if previously blocked
 // this->driver_.unlock();

  return true;
}

/*  [action callbacks] */

/*  [action requests] */

void HokuyoLaser3dDriverNode::postNodeOpenHook(void)
{
}

void HokuyoLaser3dDriverNode::addNodeDiagnostics(void)
{
}

void HokuyoLaser3dDriverNode::addNodeOpenedTests(void)
{
}

void HokuyoLaser3dDriverNode::addNodeStoppedTests(void)
{
}

void HokuyoLaser3dDriverNode::addNodeRunningTests(void)
{
}

void HokuyoLaser3dDriverNode::reconfigureNodeHook(int level)
{
}

HokuyoLaser3dDriverNode::~HokuyoLaser3dDriverNode()
{
  //[free dynamic memory]
}

/* main function */
int main(int argc,char *argv[])
{
  return driver_base::main<HokuyoLaser3dDriverNode>(argc,argv,"hokuyo_laser3d_driver_node");
}

void HokuyoLaser3dDriverNode::createPointCloud(clouddataxyz xyz_scan)
{
  this->PointCloud_msg_.header.stamp = ros::Time::now();//xyz_scan->start_timestamp;
  this->PointCloud_msg_.header.frame_id = this->driver_.config_.frame_id;
  this->PointCloud_msg_.header.seq = 1;


  this->PointCloud_msg_.points.resize(xyz_scan.points.size());
  this->PointCloud_msg_.channels.resize(1);
  this->PointCloud_msg_.channels[0].name = "intensities";
  this->PointCloud_msg_.channels[0].values.resize(xyz_scan.i.size());

  for(uint i=0;i<xyz_scan.points.size();i++)
  {
    this->PointCloud_msg_.points[i].x = xyz_scan.points[i].x/1000;
    this->PointCloud_msg_.points[i].y = xyz_scan.points[i].y/1000;
    this->PointCloud_msg_.points[i].z = xyz_scan.points[i].z/1000;
    if(xyz_scan.i.size()>0)
      this->PointCloud_msg_.channels[0].values[i] = xyz_scan.points[i].i;
  }


}

void HokuyoLaser3dDriverNode::createPointCloud2(clouddataxyz xyz_scan, sensor_msgs::PointCloud2 & pointcloud)
{
  // Header
  pointcloud.header.frame_id = this->driver_.config_.frame_id;
  pointcloud.header.stamp = ros::Time::now ();


  // 2D structure of the point cloud. If the cloud is unordered, height is
  // 1 and width is the length of the point cloud.
  pointcloud.height = 1;
  pointcloud.width = xyz_scan.points.size();

  pointcloud.fields.resize(5);
  pointcloud.fields[0].name = "x";
  pointcloud.fields[1].name = "y";
  pointcloud.fields[2].name = "z";
  pointcloud.fields[3].name = "intensity";
  pointcloud.fields[4].name = "distance";

  // Set all the fields types accordingly
  int offset = 0;
  for (size_t s = 0; s < pointcloud.fields.size(); ++s, offset += sizeof(float))
  {
    pointcloud.fields[s].offset   = offset;
    pointcloud.fields[s].count    = 1;
    pointcloud.fields[s].datatype = sensor_msgs::PointField::FLOAT32;
  }

  pointcloud.point_step = offset; // size of a point in bytes (sizeof(float)*fields.size())
  pointcloud.row_step   = pointcloud.point_step*pointcloud.width;
  pointcloud.is_dense   = false;
  pointcloud.data.resize(pointcloud.row_step*pointcloud.height);

  for (uint v = 0; v < pointcloud.height; v++)
  {
    for (uint u = 0; u < pointcloud.width; u++)
    {
      int index = v * pointcloud.width + u;
      float *pstep = (float*)&pointcloud.data[(index) * pointcloud.point_step];
      pstep[0] = xyz_scan.points[index].x/1000;
      pstep[1] = xyz_scan.points[index].y/1000;
      pstep[2] = xyz_scan.points[index].z/1000;
      pstep[3] = xyz_scan.points[index].i;
      pstep[4] = xyz_scan.points[index].d/1000;
    }
  }

}
