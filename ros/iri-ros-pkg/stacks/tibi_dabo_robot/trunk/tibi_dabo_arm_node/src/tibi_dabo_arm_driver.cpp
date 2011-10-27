#include "tibi_dabo_arm_driver.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>

std::string arm_name="tibi_dabo_arm";
std::string arm_motion_sequence_name="arm_motion_sequence";
std::string default_config_file="base_arm_config.xml";
std::string default_motion_file="motion1.xml";
std::string node_path=std::string(getenv("IRI_ROS_STACK_PATH"))+std::string("/tibi_dabo_robot/tibi_dabo_arm_node/");
std::string config_path=std::string("xml/config/");
std::string motion_path=std::string("xml/motion/");

TibiDaboArmDriver::TibiDaboArmDriver():arm_driver(arm_name)
{
  //setDriverId(driver string id);
  if(chdir(node_path.c_str())!=0)
    std::cout << "error changing the directory" << std::endl;
  // list all the existing XML files
  this->scan_XML_files();

  //setDriverId(driver string id);
  this->motion_sequence=NULL;
  this->feedback_rate=10;

  this->arm_config_file=config_path + default_config_file;
//  this->motion_seq_file=default_motion_file;
//  this->pan_angle=0.0;
//  this->tilt_angle=0.0;
//  this->pan_speed=0.0;
//  this->tilt_speed=0.0;
}

void TibiDaboArmDriver::scan_XML_files(void)
{
  std::string full_path;
  struct dirent *dp;
  struct stat st;
  DIR *dir;

  // check wether the config XML files folder exists or not
  if(stat(config_path.c_str(),&st)==0)// the folder exists
  {
    full_path=node_path + config_path;
    dir=opendir(full_path.c_str());
    while((dp=readdir(dir)) != NULL)
    {
      if(strstr(dp->d_name,".xml")!=NULL)
        this->config_files.push_back(dp->d_name);
    }
    closedir(dir);
  }
  if(stat(motion_path.c_str(),&st)==0)// the folder exists
  {
    full_path=node_path + motion_path;
    dir=opendir(full_path.c_str());
    while((dp=readdir(dir)) != NULL)
    {
      if(strstr(dp->d_name,".xml")!=NULL)
        this->motion_seq_files.push_back(dp->d_name);
    }
    closedir(dir);
  }
}

bool TibiDaboArmDriver::openDriver(void)
{
//  std::vector<float> position;
  std::vector<bool> enabled;
  unsigned int i=0;

  //setDriverId(driver string id);
  this->lock();
  try{
    this->arm_driver.load_config(this->arm_config_file);
    for(i=0;i<this->arm_driver.get_num_motors();i++)
      enabled.push_back(true);
    this->arm_driver.enable(enabled);
//    this->arm_driver.get_position(position);
//    this->pan_angle=position[0];
//    this->tilt_angle=position[1];
  }catch(CException &e){
    std::cout << e.what() << std::endl;
    ROS_INFO("Impossible to configure the arm with the given parameters");
    this->unlock();
    return false;
  }
  this->unlock();
  return true;
}

bool TibiDaboArmDriver::closeDriver(void)
{
  return true;
}

bool TibiDaboArmDriver::startDriver(void)
{
  return true;
}

bool TibiDaboArmDriver::stopDriver(void)
{
  return true;
}

void TibiDaboArmDriver::config_update(Config& new_cfg, uint32_t level)
{
  std::vector<bool> absolute(2);
  std::string motion_seq;
  unsigned int i=0;
//  static bool first=true;

  this->lock();
 
  // depending on current state
  // update driver with new_cfg data
  switch(this->getState())
  {
    case TibiDaboArmDriver::CLOSED:
      this->arm_config_file=config_path + new_cfg.config_file;
      break;

    case TibiDaboArmDriver::OPENED:
      break;

    case TibiDaboArmDriver::RUNNING:
      // stop the current motion if necessary
      if(this->motion_sequence!=NULL)
      {
        if(this->motion_sequence->get_current_state()==mtn_started ||
           this->motion_sequence->get_current_state()==mtn_paused)
          this->motion_sequence->stop_sequence();
      }
      // configure the robot
      if(new_cfg.control_mode==0)// position control
        this->arm_driver.set_position_control();
      else
        this->arm_driver.set_velocity_control();
      if(new_cfg.motion_mode==0)//absolute motion
        for(i=0;i<this->arm_driver.get_num_motors();i++)
          absolute[i]=true;
      else
        for(i=0;i<this->arm_driver.get_num_motors();i++)
          absolute[i]=false;
      this->arm_driver.set_absolute_motion(absolute);
      // update the feedback rate
      this->feedback_rate=new_cfg.feedback_rate;
/*      try{
        if(first)
        {
          this->arm_driver.get_position(position);
          new_cfg.pan_angle=position[0];
          new_cfg.tilt_angle=position[1];
          this->pan_angle=position[0];
          this->tilt_angle=position[1];
          first=false;
          this->unlock();
        }
        else
        {
          if(new_cfg.pan_angle!=this->pan_angle || new_cfg.tilt_angle!=this->tilt_angle)
          {
            this->pan_angle=new_cfg.pan_angle;
            position[0]=this->pan_angle;
            this->pan_speed=new_cfg.pan_speed;
            velocity[0]=this->pan_speed;
            acceleration[0]=velocity[0]*velocity[0]/(0.25*position[0]);
            this->tilt_angle=new_cfg.tilt_angle;
            position[1]=this->tilt_angle;
            this->tilt_speed=new_cfg.tilt_speed;
            velocity[1]=this->tilt_speed;
            acceleration[1]=velocity[1]*velocity[1]/(0.25*position[1]);
            this->stop();
            this->unlock();
            this->move(position,velocity,acceleration);
          }
          else if(this->motion_seq_file!=new_cfg.motion_file)
          {
            this->motion_seq_file=new_cfg.motion_file;
            motion_seq=motion_path + this->motion_seq_file;
            this->start_motion_sequence(motion_seq);
            this->unlock();
          }
        }
      }catch(CException &e){
        std::cout << e.what() << std::endl;
        this->unlock();
      }*/
      break;
  }

  // save the current configuration
  this->config_=new_cfg;
  this->unlock();

}

// API for the sequences
void TibiDaboArmDriver::start_motion_sequence(std::string &filename)
{
  try{
    if(this->motion_sequence!=NULL)
    {
      delete this->motion_sequence;
      this->motion_sequence=NULL;
      this->arm_driver.stop();
    }
    this->motion_sequence=new CMotionSequence(arm_motion_sequence_name);
    this->motion_sequence->set_motor_group(&this->arm_driver);
    this->motion_sequence->set_timeout_factor(1.5);
    this->motion_sequence->load_sequence(filename);
    this->motion_sequence->start_sequence();
  }catch(CException &e){
    std::cout << e.what() << std::endl;
  }
}

void TibiDaboArmDriver::start_motion_sequence(std::vector<TMotionStep> &seq)
{
  unsigned int i=0;

  try{
    if(this->motion_sequence!=NULL)
    {
      delete this->motion_sequence;
      this->motion_sequence=NULL;
      this->arm_driver.stop();
    }
    this->motion_sequence=new CMotionSequence(arm_motion_sequence_name);
    this->motion_sequence->set_motor_group(&this->arm_driver);
    this->motion_sequence->set_timeout_factor(1.5);
    for(i=0;i<seq.size();i++)
      this->motion_sequence->add_step(seq[i].position,seq[i].velocity,seq[i].delay);
    this->motion_sequence->start_sequence();
  }catch(CException &e){
    std::cout << e.what() << std::endl;
  }
}

void TibiDaboArmDriver::pause_motion_sequence(void)
{
  this->motion_sequence->pause_sequence();
}

void TibiDaboArmDriver::resume_motion_sequence(void)
{
  this->motion_sequence->resume_sequence();
}

void TibiDaboArmDriver::stop_motion_sequence(void)
{
  this->motion_sequence->stop_sequence();
}

std::string TibiDaboArmDriver::get_motion_sequence_complete_event_id(void)
{
  if(this->motion_sequence!=NULL)
    return this->motion_sequence->get_sequence_complete_event_id();
  else
    return std::string("");
}

std::string TibiDaboArmDriver::get_motion_sequence_error_event_id(void)
{
  if(this->motion_sequence!=NULL)
    return this->motion_sequence->get_sequence_error_event_id();
  else
    return std::string("");
}

std::string TibiDaboArmDriver::get_motion_sequence_error_message(void)
{
  if(this->motion_sequence!=NULL)
    return this->motion_sequence->get_error_message();
  else
    return std::string("");
}

float TibiDaboArmDriver::get_motion_sequence_completed_percentage(void)
{
  if(this->motion_sequence!=NULL)
    return this->motion_sequence->get_completed_percentage();
  else
    return 0.0;
}

// API for the discete motions
std::string TibiDaboArmDriver::get_position_reached_event_id(void)
{
  return this->arm_driver.get_position_reached_event_id();
}

void TibiDaboArmDriver::move(std::vector<float> &position,std::vector<float> &velocity,std::vector<float> &acceleration)
{
  CEventServer *event_server=CEventServer::instance();
  std::list<std::string> events;

  try{
    if(this->motion_sequence!=NULL)
    {
      delete this->motion_sequence;
      this->motion_sequence=NULL;
      this->arm_driver.move(position,velocity,acceleration);
    }
    else
    {
      events.push_back(this->arm_driver.get_position_reached_event_id());
      event_server->wait_all(events);
      this->arm_driver.move(position,velocity,acceleration);
    }
    sleep(1);
  }catch(CException &e){
    std::cout << e.what() << std::endl;
    throw;
  }
}

void TibiDaboArmDriver::stop(void)
{
  this->arm_driver.stop();
}

void TibiDaboArmDriver::get_position(std::vector<float> &position)
{
  this->arm_driver.get_position(position);
}

void TibiDaboArmDriver::get_velocity(std::vector<float> &velocity)
{
  this->arm_driver.get_velocity(velocity);
}

void TibiDaboArmDriver::set_absolute_motion(bool status)
{
  std::vector<bool> absolute;
  unsigned int i=0;

  try{
    for(i=0;i<this->arm_driver.get_num_motors();i++)
      absolute.push_back(status);
    this->arm_driver.set_absolute_motion(absolute);
  }catch(CException &e){
    std::cout << e.what() << std::endl;
    throw;
  }
}

void TibiDaboArmDriver::set_position_control(bool status)
{
  try{
    if(status==true)
      this->arm_driver.set_position_control();
    else
      this->arm_driver.set_velocity_control();
  }catch(CException &e){
    std::cout << e.what() << std::endl;
    throw;
  }
}

unsigned int TibiDaboArmDriver::get_num_config_files(void)
{
  return this->config_files.size();
}

std::string TibiDaboArmDriver::get_config_file(unsigned int index)
{
  if(index > this->config_files.size())
  {
    /* handle exceptions */
  }
  return this->config_files[index];
}

unsigned int TibiDaboArmDriver::get_num_motion_seq_files(void)
{
  return this->motion_seq_files.size();
}

std::string TibiDaboArmDriver::get_motion_seq_file(unsigned int index)
{
  if(index > this->motion_seq_files.size())
  {
    /* handle exceptions */
  }
  return this->motion_seq_files[index];
}

int TibiDaboArmDriver::get_feedback_rate(void)
{
  return this->feedback_rate;
}

TibiDaboArmDriver::~TibiDaboArmDriver()
{
}
