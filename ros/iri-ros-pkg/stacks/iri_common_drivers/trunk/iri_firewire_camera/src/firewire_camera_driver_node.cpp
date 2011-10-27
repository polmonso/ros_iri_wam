#include "firewire_camera_driver_node.h"

FirewireCameraDriverNode::FirewireCameraDriverNode(ros::NodeHandle &nh) : iri_base_driver::IriBaseNodeDriver<FirewireCameraDriver>(nh), camera_manager(ros::NodeHandle(this->public_node_handle_))
{
  //init class attributes if necessary
  this->loop_rate_ = 1000;//in [Hz]
  this->desired_framerate=30.0;
  this->diagnosed_camera_image=NULL;
  this->it=NULL;

  this->it=new image_transport::ImageTransport(this->public_node_handle_);
  this->diagnosed_camera_image = new diagnostic_updater::HeaderlessTopicDiagnostic("camera_image",this->diagnostic_,
                                     diagnostic_updater::FrequencyStatusParam(&this->desired_framerate,&this->desired_framerate,0.01,5));

  this->event_server=CEventServer::instance();

  // [init publishers]
  this->camera_image_publisher_ = this->it->advertiseCamera("camera_image", 1);

  
  // [init subscribers]
  
  // [init services]
  this->camera_config_server_ = this->public_node_handle_.advertiseService("camera_config", &FirewireCameraDriverNode::camera_configCallback, this);
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

void FirewireCameraDriverNode::mainNodeThread(void)
{
  std::list<std::string> events;
  char *image_data=NULL;
  TCameraConfig config;
  unsigned int i=0;

  try{
    //lock access to driver if necessary
    this->driver_.lock();
    if(this->driver_.isRunning())
    {
    this->new_frame_event=this->driver_.get_new_frame_event();
    if(this->new_frame_event!="")
    {
      events.push_back(this->new_frame_event);
      this->event_server->wait_all(events,1000);
      this->driver_.get_image(&image_data);
      if(image_data!=NULL)
      {
        this->driver_.get_current_config(&config);
        // update the desired framerate for the diagnostics
        this->desired_framerate=config.framerate;
        //fill msg structures
        this->Image_msg_.width=config.width;
        this->Image_msg_.height=config.height;
        this->Image_msg_.step=config.width*config.depth/8;
        this->Image_msg_.data.resize(config.width*config.height*config.depth/8);
        for(i=0;i<config.width*config.height*config.depth/8;i++)
          this->Image_msg_.data[i]=image_data[i];
        if(config.coding==MONO || config.coding==RAW)
        {
          if(config.depth==DEPTH8)
            this->Image_msg_.encoding="8UC1";
          else if(config.depth==DEPTH16)
            this->Image_msg_.encoding="16UC1";
          else
            this->Image_msg_.encoding="16UC1";
        }
        else
        {
          if(config.depth==DEPTH24)
            this->Image_msg_.encoding="rgb8";
          else if(config.depth==DEPTH48)
            this->Image_msg_.encoding="16UC3";
          else
            this->Image_msg_.encoding="16UC3";
        }
        delete[] image_data;
      }
    }
    }
    this->driver_.unlock();
  }catch(CException &e){
    this->driver_.unlock();
    ROS_INFO("Impossible to capture frame");
  }
  //fill srv structure and make request to the server

  //publish messages
  this->Image_msg_.header.stamp = ros::Time::now();
  //this->Image_msg_.header.frame_id = "<publisher_topic_name>";
  this->camera_image_publisher_.publish(this->Image_msg_,this->camera_manager.getCameraInfo());
  this->diagnosed_camera_image->tick();
}
  // [fill msg Header if necessary]

void FirewireCameraDriverNode::check_configuration(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  TCameraConfig desired_config;
  TCameraConfig current_config;

  this->driver_.get_current_config(&current_config);
  this->driver_.get_desired_config(&desired_config);

  if(current_config.left_offset!=desired_config.left_offset)
  {
    stat.summary(1,"The current configuration differs from the desired configuration");
    stat.addf("Image left offset:","current value is %d,desired value is %d", current_config.left_offset,desired_config.left_offset);
  }
  else if(current_config.top_offset!=desired_config.top_offset)
  {
    stat.summary(1,"The current configuration differs from the desired configuration");
    stat.addf("Image top offset:","current value is %d,desired value is %d", current_config.top_offset,desired_config.top_offset);
  }
  else if(current_config.width!=desired_config.width)
  {
    stat.summary(1,"The current configuration differs from the desired configuration");
    stat.addf("Image width:","current value is %d,desired value is %d", current_config.width,desired_config.width);
  }
  else if(current_config.height!=desired_config.height)
  {
    stat.summary(1,"The current configuration differs from the desired configuration");
    stat.addf("Image height:","current value is %d,desired value is %d", current_config.height,desired_config.height);
  }
  else
    stat.summary(0,"The current configuration is okay");
}

/*  [subscriber callbacks] */

/*  [service callbacks] */
bool FirewireCameraDriverNode::camera_configCallback(iri_common_msgs::CameraConfig::Request &req, iri_common_msgs::CameraConfig::Response &res) 
{ 
  //lock access to driver if necessary 
  //this->driver_.lock(); 

  //if(this->driver_.isRunning()) 
  //{ 
    //do operations with req and output on res 
    //res.data2 = req.data1 + my_var; 
  //} 
  //else 
  //{ 
    //std::cout << "ERROR: Driver is not on run mode yet." << std::endl; 
  //} 

  //unlock driver if previously blocked 
  //this->driver_.unlock(); 

  return true; 
}

/*  [action callbacks] */

/*  [action requests] */

void FirewireCameraDriverNode::postNodeOpenHook(void)
{
}

void FirewireCameraDriverNode::addNodeDiagnostics(void)
{
  diagnostic_.add("Check Configuration", this, &FirewireCameraDriverNode::check_configuration);
}

void FirewireCameraDriverNode::addNodeOpenedTests(void)
{
}

void FirewireCameraDriverNode::addNodeStoppedTests(void)
{
}

void FirewireCameraDriverNode::addNodeRunningTests(void)
{
}

void FirewireCameraDriverNode::reconfigureNodeHook(int level)
{
}

FirewireCameraDriverNode::~FirewireCameraDriverNode()
{
  // [free dynamic memory]
  if(this->it!=NULL)
  {
    delete this->it;
    this->it=NULL;
  }
  if(this->diagnosed_camera_image!=NULL)
  {
    delete this->diagnosed_camera_image;
    this->diagnosed_camera_image=NULL;
  }
}

/* main function */
int main(int argc,char *argv[])
{
  return driver_base::main<FirewireCameraDriverNode>(argc,argv,"firewire_camera_driver_node");
}
