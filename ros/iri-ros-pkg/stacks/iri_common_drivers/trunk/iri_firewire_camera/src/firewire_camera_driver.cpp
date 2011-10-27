#include "firewire_camera_driver.h"

FirewireCameraDriver::FirewireCameraDriver()
{
  //setDriverId(driver string id);
  this->server=CFirewireServer::instance();
  this->server->init();
  this->camera=NULL;
  this->camera_id=-1;
  this->ISO_speed=800;
  // desired configuration
  this->desired_conf.width=-1;
  this->desired_conf.height=-1;
  this->desired_conf.left_offset=-1;
  this->desired_conf.top_offset=-1;
  this->desired_conf.depth=(depths_t)-1;
  this->desired_conf.coding=(codings_t)-1;
  this->desired_conf.framerate=0.0;
  // current configuration
  this->current_conf.width=-1;
  this->current_conf.height=-1;
  this->current_conf.left_offset=-1;
  this->current_conf.top_offset=-1;
  this->current_conf.depth=(depths_t)-1;
  this->current_conf.coding=(codings_t)-1;
  this->current_conf.framerate=0.0;
  // default configuration
  this->default_conf.width=-1;
  this->default_conf.height=-1;
  this->default_conf.left_offset=-1;
  this->default_conf.top_offset=-1;
  this->default_conf.depth=(depths_t)-1;
  this->default_conf.coding=(codings_t)-1;
  this->default_conf.framerate=0.0;
}

bool FirewireCameraDriver::openDriver(void)
{
  //setDriverId(driver string id);
  if(this->camera_id!=-1)
  {
    this->server->init();
    if(this->server->get_num_cameras()>0)
    {
      this->server->get_camera(this->camera_id,&this->camera);
      this->camera->set_ISO_speed(this->ISO_speed);
      this->camera->get_config(&this->default_conf.left_offset,&this->default_conf.top_offset,&this->default_conf.width,
                               &this->default_conf.height,&this->default_conf.framerate,&this->default_conf.depth,
                               &this->default_conf.coding);
      this->change_config(&this->default_conf);
      ROS_INFO("Driver opened");
      return true;
    }
    else
    {   
      ROS_INFO("No cameras available");
      return false;
    }
  }
  else
  {
    ROS_INFO("Waiting for a valid id ...");
    return false;
  
  }
}

bool FirewireCameraDriver::closeDriver(void)
{ 
  if(this->camera!=NULL)
  {  
    delete this->camera;
    this->camera=NULL;
  }

  return true;
}

bool FirewireCameraDriver::startDriver(void)
{
  if(this->camera!=NULL)
  {
    try{
      this->lock();
      this->camera->start();
      this->unlock();
      ROS_INFO("Driver started");
      return true;
    }catch(CException &e){
      this->unlock();
      //ROS_INFO(e.what().data());
      return false;
    }
  }
  else
    return false;
}

bool FirewireCameraDriver::stopDriver(void)
{
  if(this->camera!=NULL)
  {
    try{
      this->lock();
      this->camera->stop();
      this->unlock();
      ROS_INFO("Driver stopped");
      return true;
    }catch(CException &e){
      this->unlock();
      //ROS_INFO(e.what().data());
      return false;
    }
  }
  else
    return false;
}

void FirewireCameraDriver::config_update(const Config& new_cfg, uint32_t level)
{
  TCameraConfig conf;

  std::cout << "dynamic reconfigure" << std::endl; 
  // depending on current state
  // update driver with new_cfg data
  switch(this->getState())
  {
    case FirewireCameraDriver::CLOSED:
      this->lock();
      this->camera_id=new_cfg.Camera_node;
      this->unlock();
      break;

    case FirewireCameraDriver::OPENED:
      if(new_cfg.Camera_node!=-1)
      {
        try{
          this->lock();
          if(this->camera_id!=new_cfg.Camera_node)
          {
            this->close();
            this->camera_id=new_cfg.Camera_node;
            this->open();
          }
          conf.left_offset=new_cfg.Left_offset;
          conf.top_offset=new_cfg.Top_offset;
          conf.width=new_cfg.Image_width;
          conf.height=new_cfg.Image_height;
          conf.framerate=new_cfg.Framerate;
          switch(new_cfg.Color_coding)
          {
            case 0: conf.depth=DEPTH8;
                    conf.coding=MONO;
                    break;
            case 1: conf.depth=DEPTH8;
                    conf.coding=YUV;
                    break;
            case 2: conf.depth=DEPTH16;
                    conf.coding=YUV;
                    break;
            case 3: conf.depth=DEPTH24;
                    conf.coding=RGB;
                    break;
            case 4: conf.depth=DEPTH16;
                    conf.coding=MONO;
                    break;
            case 5: conf.depth=DEPTH48;
                    conf.coding=RGB;
                    break;
            case 6: conf.depth=DEPTH8;
                    conf.coding=RAW;
                    break;
            case 7: conf.depth=DEPTH16;
                    conf.coding=RAW;
                    break;
            default: conf.depth=DEPTH24;
                     conf.coding=RGB;
                     break;
          }
          this->change_config(&conf);
          // white balance feature
          this->set_white_balance(new_cfg.White_balance_enabled,new_cfg.White_balance_mode,new_cfg.White_balance_u_b_value,
                                  new_cfg.White_balance_v_r_value);
          // shutter feature
          this->set_shutter(new_cfg.Shutter_enabled,new_cfg.Shutter_mode,new_cfg.Shutter_value);
          // gain feature
          this->set_gain(new_cfg.Gain_enabled,new_cfg.Gain_mode,new_cfg.Gain_value);
          this->unlock();
        }catch(CFirewireCameraException &e){
          ROS_INFO("Invalid confoguration. Setting back the default configuration ...");
          this->change_config(&this->default_conf);
          this->unlock();
        }
      }
      break;

    case FirewireCameraDriver::RUNNING:
      break;
  }

  // save the current configuration
  this->config_=new_cfg;
}

void FirewireCameraDriver::change_config(TCameraConfig *new_conf)
{
  try{
    if(this->camera!=NULL)
    {
      this->current_conf.left_offset=new_conf->left_offset;
      this->current_conf.top_offset=new_conf->top_offset;
      this->current_conf.width=new_conf->width;
      this->current_conf.height=new_conf->height;
      this->current_conf.framerate=new_conf->framerate;
      this->current_conf.depth=new_conf->depth;
      this->current_conf.coding=new_conf->coding;
      this->camera->set_config(&this->current_conf.left_offset,&this->current_conf.top_offset,&this->current_conf.width,
                               &this->current_conf.height,&this->current_conf.framerate,this->current_conf.depth,
                               this->current_conf.coding);
      this->desired_conf.left_offset=new_conf->left_offset;
      this->desired_conf.top_offset=new_conf->top_offset;
      this->desired_conf.width=new_conf->width;
      this->desired_conf.height=new_conf->height;
      this->desired_conf.framerate=new_conf->framerate;
      this->desired_conf.depth=new_conf->depth;
      this->desired_conf.coding=new_conf->coding;
    }
  }catch(CFirewireCameraException &e){
    ROS_INFO("Invalid confoguration. Setting back the default configuration ...");
    this->change_config(&this->default_conf);
  }catch(CFirewireInternalException &e){
    throw;
  }
}

void FirewireCameraDriver::set_white_balance(bool enable,int mode, int u_b_value, int v_r_value)
{
  try{
    if(enable)
    {
      if(!this->camera->is_feature_enabled(DC1394_FEATURE_WHITE_BALANCE))
        this->camera->enable_feature(DC1394_FEATURE_WHITE_BALANCE);
      if(mode==0)
        this->camera->set_feature_auto(DC1394_FEATURE_WHITE_BALANCE);
      else
      {
        this->camera->set_feature_manual(DC1394_FEATURE_WHITE_BALANCE);
        this->camera->set_white_balance_value(u_b_value,v_r_value); 
      }
    }
    else
    {
      if(this->camera->is_feature_enabled(DC1394_FEATURE_WHITE_BALANCE))
        this->camera->disable_feature(DC1394_FEATURE_WHITE_BALANCE);
    }
  }catch(CFirewireInternalException &e){
    ROS_INFO("Impossible to set the white balance feature");
  }catch(CFirewireFeatureException &e){
    ROS_INFO("Impossible to set the white balance feature");
  }
}

void FirewireCameraDriver::set_shutter(bool enable,int mode, int value)
{
  try{
    if(enable)
    {
      if(!this->camera->is_feature_enabled(DC1394_FEATURE_SHUTTER))
        this->camera->enable_feature(DC1394_FEATURE_SHUTTER);
      if(mode==0)
        this->camera->set_feature_auto(DC1394_FEATURE_SHUTTER);
      else
      {
        this->camera->set_feature_manual(DC1394_FEATURE_SHUTTER);
        this->camera->set_feature_value(DC1394_FEATURE_SHUTTER,value);
      }
    }
    else
    {
      if(this->camera->is_feature_enabled(DC1394_FEATURE_SHUTTER))
        this->camera->disable_feature(DC1394_FEATURE_SHUTTER);
    }
  }catch(CFirewireInternalException &e){
    ROS_INFO("Impossible to set the shutter feature");
  }
  catch(CFirewireFeatureException &e){
    ROS_INFO("Impossible to set the shutter feature");
  }
}

void FirewireCameraDriver::set_gain(bool enable, int mode, int value)
{
  try{
    if(enable)
    {
      if(!this->camera->is_feature_enabled(DC1394_FEATURE_GAIN))
        this->camera->enable_feature(DC1394_FEATURE_GAIN);
      if(mode==0)
        this->camera->set_feature_auto(DC1394_FEATURE_GAIN);
      else
      {
        this->camera->set_feature_manual(DC1394_FEATURE_GAIN);
        this->camera->set_feature_value(DC1394_FEATURE_GAIN,value);
      }
    }
    else
    {
      if(this->camera->is_feature_enabled(DC1394_FEATURE_GAIN))
        this->camera->disable_feature(DC1394_FEATURE_GAIN);
    }
  }catch(CFirewireInternalException &e){
    ROS_INFO("Impossible to set the gain feature");
  }
  catch(CFirewireFeatureException &e){
    ROS_INFO("Impossible to set the gain feature");
  }
}

void FirewireCameraDriver::get_image(char **image_data)
{
  if(this->camera!=NULL)
    this->camera->get_image(image_data);
  else
    *image_data=NULL;
}

void FirewireCameraDriver::set_ISO_speed(int iso_speed)
{
  this->ISO_speed=iso_speed;
}

void FirewireCameraDriver::set_config(TCameraConfig *new_conf)
{
  try{
    this->lock();
    this->change_config(new_conf);
    this->unlock();
  }catch(CException &e){
    this->unlock();
    throw;
  }
}

void FirewireCameraDriver::get_current_config(TCameraConfig *current_conf)
{
  current_conf->left_offset=this->current_conf.left_offset;
  current_conf->top_offset=this->current_conf.top_offset;
  current_conf->width=this->current_conf.width;
  current_conf->height=this->current_conf.height;
  current_conf->framerate=this->current_conf.framerate;
  current_conf->depth=this->current_conf.depth;
  current_conf->coding=this->current_conf.coding;
}

void FirewireCameraDriver::get_desired_config(TCameraConfig *desired_conf)
{
  desired_conf->left_offset=this->desired_conf.left_offset;
  desired_conf->top_offset=this->desired_conf.top_offset;
  desired_conf->width=this->desired_conf.width;
  desired_conf->height=this->desired_conf.height;
  desired_conf->framerate=this->desired_conf.framerate;
  desired_conf->depth=this->desired_conf.depth;
  desired_conf->coding=this->desired_conf.coding;
}

void FirewireCameraDriver::get_default_config(TCameraConfig *default_conf)
{
  default_conf->left_offset=this->default_conf.left_offset;
  default_conf->top_offset=this->default_conf.top_offset;
  default_conf->width=this->default_conf.width;
  default_conf->height=this->default_conf.height;
  default_conf->framerate=this->default_conf.framerate;
  default_conf->depth=this->default_conf.depth;
  default_conf->coding=this->default_conf.coding;
}

std::string FirewireCameraDriver::get_new_frame_event(void)
{  
  std::string event="";

  if(this->camera!=NULL) 
    this->camera->get_new_frame_event_id(event);

  return event;
}

FirewireCameraDriver::~FirewireCameraDriver()
{
  this->server->close();
}
