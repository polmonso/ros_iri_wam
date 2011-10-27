#include "tibi_dabo_head_driver.h"
#include "exceptions.h"
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>

std::string head_name="tibi_dabo_head";
std::string head_motion_sequence_name="head_motion_sequence";
std::string head_lights_sequence_name="head_lights_sequence";
std::string default_config_file="head_config.xml";
std::string default_motion_file="head_home.xml";
std::string default_lights_file="home.xml";
std::string node_path=std::string(getenv("IRI_ROS_STACK_PATH"))+std::string("/tibi_dabo_robot/tibi_dabo_head_node/");
std::string config_path=std::string("xml/config/");
std::string motion_path=std::string("xml/motion/");
std::string lights_path=std::string("xml/lights/");

TibiDaboHeadDriver::TibiDaboHeadDriver()
{
  if(chdir(node_path.c_str())!=0)
    ROS_INFO("error changing the directory");
  // list all the existing XML files
  this->scan_XML_files(); 

  //setDriverId(driver string id);
  this->motion_sequence=NULL;
  this->lights_sequence=NULL;
  this->head_driver=NULL;
  this->feedback_rate=10;

  this->head_config_file=config_path + default_config_file;
//  this->motion_seq_file=default_motion_file;
//  this->lights_seq_file=default_lights_file;
//  this->position.resize(3);
//  this->velocity.resize(3);
//  this->facial_expression_id=-1;
//  this->brightness=50.0;
}

void TibiDaboHeadDriver::scan_XML_files(void)
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
  if(stat(lights_path.c_str(),&st)==0)// the folder exists
  {
    full_path=node_path + lights_path;
    dir=opendir(full_path.c_str());
    while((dp=readdir(dir)) != NULL) 
    {
      if(strstr(dp->d_name,".xml")!=NULL)
        this->lights_seq_files.push_back(dp->d_name);
    }
    closedir(dir);
  }
}

bool TibiDaboHeadDriver::openDriver(void)
{
  std::vector<bool> enabled;
  unsigned int i=0;

  //setDriverId(driver string id);
  try{
    this->lock();
    this->head_driver=new CSegwayHead(head_name);
    this->head_driver->load_config(this->head_config_file);
    for(i=0;i<this->head_driver->get_num_motors();i++)
      enabled.push_back(true);
    this->head_driver->enable(enabled);
//    this->head_driver->get_position(this->position);
    this->unlock();
    return true;
  }catch(CException &e){
    if(this->head_driver!=NULL)
    { 
      delete this->head_driver;
      this->head_driver=NULL;
    }
    std::cout << e.what() << std::endl;
    ROS_INFO("Impossible to configure the head with the given parameters");
    this->unlock();
    return false;
  }

  return false;
}

bool TibiDaboHeadDriver::closeDriver(void)
{
  if(this->head_driver!=NULL)
  {
    delete this->head_driver;
    this->head_driver=NULL;
  }

  return true;
}

bool TibiDaboHeadDriver::startDriver(void)
{
  return true;
}

bool TibiDaboHeadDriver::stopDriver(void)
{
  return true;
}

void TibiDaboHeadDriver::config_update(Config& new_cfg, uint32_t level)
{
  std::vector<bool> absolute(3);
  unsigned int i=0;
//  static bool first=true;

  this->lock();
  
  // depending on current state
  // update driver with new_cfg data
  switch(this->getState())
  {
    case TibiDaboHeadDriver::CLOSED:
      this->head_config_file=config_path + new_cfg.config_file;
      break;

    case TibiDaboHeadDriver::OPENED:
      break;

    case TibiDaboHeadDriver::RUNNING:
      // stop the current motion if necessary
      if(this->motion_sequence!=NULL)
      {
        if(this->motion_sequence->get_current_state()==mtn_started ||
           this->motion_sequence->get_current_state()==mtn_paused)
          this->motion_sequence->stop_sequence();
      }
      // configure the robot
      if(this->head_driver!=NULL)
      {
        if(new_cfg.control_mode==0)// position control
          this->head_driver->set_position_control();
        else
          this->head_driver->set_velocity_control();
        if(new_cfg.motion_mode==0)//absolute motion
          for(i=0;i<this->head_driver->get_num_motors();i++)
            absolute[i]=true;
        else 
          for(i=0;i<this->head_driver->get_num_motors();i++)
            absolute[i]=false;
        this->head_driver->set_absolute_motion(absolute);
      }
      // update the feedback rate
      this->feedback_rate=new_cfg.feedback_rate;
/*      try{
        if(first)
        {
          if(this->head_driver!=NULL)
          {
            this->head_driver->get_position(this->position);
            new_cfg.pan_angle=this->position[0];
            new_cfg.tilt_angle=this->position[1];
            new_cfg.side_angle=this->position[2];
            first=false;
          }
        }
        else
        {
          if(new_cfg.pan_angle!=this->position[0] || new_cfg.tilt_angle!=this->position[1] || new_cfg.side_angle!=this->position[2])
          { 
            this->position[0]=new_cfg.pan_angle;
            this->velocity[0]=new_cfg.pan_speed;
            acceleration[0]=this->velocity[0]*this->velocity[0]/(0.25*this->position[0]);
            this->position[1]=new_cfg.tilt_angle;
            this->velocity[1]=new_cfg.tilt_speed;
            acceleration[1]=this->velocity[1]*this->velocity[1]/(0.25*this->position[1]);
            this->position[2]=new_cfg.side_angle;
            this->velocity[2]=new_cfg.side_speed;
            acceleration[2]=this->velocity[2]*this->velocity[2]/(0.25*this->position[2]);
            this->move(position,velocity,acceleration);
          }  
          else if(this->motion_seq_file!=new_cfg.motion_seq_file)
          {
            this->motion_seq_file=new_cfg.motion_seq_file;
            motion_seq=motion_path + this->motion_seq_file;
            this->start_motion_sequence(motion_seq);
          }
          else if(this->lights_seq_file!=new_cfg.lights_seq_file)
          {
            this->lights_seq_file=new_cfg.lights_seq_file;
            lights_seq=lights_path + this->lights_seq_file;
            this->start_lights_sequence(lights_seq);
          }
          else if(this->facial_expression_id!=new_cfg.facial_expression)
          {
            this->facial_expression_id=new_cfg.facial_expression;
            this->set_face_expression(this->facial_expression_id);
          }  
          else if(this->brightness!=new_cfg.brightness)
          {
            this->brightness=new_cfg.brightness;
            this->set_brightness(this->brightness);
          }  
        }
      }catch(CException &e){
        std::cout << e.what() << std::endl;
      }*/
      break;
  }

  // save the current configuration
  this->config_=new_cfg;

  this->unlock();
}

// API for the sequences
void TibiDaboHeadDriver::start_motion_sequence(std::string &filename)
{
  try{
    if(this->motion_sequence!=NULL)
    {
      delete this->motion_sequence;
      this->motion_sequence=NULL;
      this->head_driver->stop();
    }
    this->motion_sequence=new CMotionSequence(head_motion_sequence_name);
    this->motion_sequence->set_motor_group(this->head_driver);
    this->motion_sequence->set_timeout_factor(1.5);
    this->motion_sequence->load_sequence(filename);
    this->motion_sequence->start_sequence();
  }catch(CException &e){
    std::cout << e.what() << std::endl;
  }
}

void TibiDaboHeadDriver::start_motion_sequence(std::vector<TMotionStep> &seq)
{
  unsigned int i=0;

  try{
    if(this->motion_sequence!=NULL)
    {
      delete this->motion_sequence;
      this->motion_sequence=NULL;
      this->head_driver->stop();
    }
    this->motion_sequence=new CMotionSequence(head_motion_sequence_name);
    this->motion_sequence->set_motor_group(this->head_driver);
    this->motion_sequence->set_timeout_factor(1.5);
    for(i=0;i<seq.size();i++)
      this->motion_sequence->add_step(seq[i].position,seq[i].velocity,seq[i].delay);
    this->motion_sequence->start_sequence();
  }catch(CException &e){
    std::cout << e.what() << std::endl;
  }
}

void TibiDaboHeadDriver::pause_motion_sequence(void)
{
  this->motion_sequence->pause_sequence();
}

void TibiDaboHeadDriver::resume_motion_sequence(void)
{
  this->motion_sequence->resume_sequence();
}

void TibiDaboHeadDriver::stop_motion_sequence(void)
{
  this->motion_sequence->stop_sequence();
}

std::string TibiDaboHeadDriver::get_motion_sequence_complete_event_id(void)
{
  if(this->motion_sequence!=NULL)
    return this->motion_sequence->get_sequence_complete_event_id();
  else 
    return std::string("");
}

std::string TibiDaboHeadDriver::get_motion_sequence_error_event_id(void)
{
  if(this->motion_sequence!=NULL)
    return this->motion_sequence->get_sequence_error_event_id();
  else
    return std::string("");
}

std::string TibiDaboHeadDriver::get_motion_sequence_error_message(void)
{
  if(this->motion_sequence!=NULL)
    return this->motion_sequence->get_error_message();
  else
    return std::string("");
}

float TibiDaboHeadDriver::get_motion_sequence_completed_percentage(void)
{
  if(this->motion_sequence!=NULL)
    return this->motion_sequence->get_completed_percentage();
  else
    return 0.0;
}

void TibiDaboHeadDriver::start_lights_sequence(std::string &filename)
{
  try{
    if(this->lights_sequence!=NULL)
    {
      delete this->lights_sequence;
      this->lights_sequence=NULL;
    }
    this->lights_sequence=new CSegwayHeadLightsSequence(head_lights_sequence_name);
    this->lights_sequence->set_segway_head(this->head_driver);
    this->lights_sequence->load_sequence(filename);
    this->lights_sequence->start_sequence();
  }catch(CException &e){
    std::cout << e.what() << std::endl;
  }
}

void TibiDaboHeadDriver::pause_lights_sequence(void)
{
  if(this->lights_sequence!=NULL)
    this->lights_sequence->pause_sequence();
}

void TibiDaboHeadDriver::resume_lights_sequence(void)
{
  if(this->lights_sequence!=NULL)
    this->lights_sequence->resume_sequence();
}

void TibiDaboHeadDriver::stop_lights_sequence(void)
{
  if(this->lights_sequence!=NULL)
    this->lights_sequence->stop_sequence();
}

std::string TibiDaboHeadDriver::get_lights_sequence_complete_event_id(void)
{
  if(this->lights_sequence!=NULL)
    return this->lights_sequence->get_sequence_complete_event_id();
  else
    return std::string("");
}

float TibiDaboHeadDriver::get_lights_sequence_completed_percentage(void)
{
  if(this->lights_sequence!=NULL)
    return this->lights_sequence->get_completed_percentage(); 
  else
    return 0.0;
}
// API for the discete motions
std::string TibiDaboHeadDriver::get_position_reached_event_id(void)
{
  if(this->head_driver!=NULL)
    return this->head_driver->get_position_reached_event_id();
  else
    return std::string("");
}

void TibiDaboHeadDriver::move(std::vector<float> &position,std::vector<float> &velocity,std::vector<float> &acceleration)
{
  CEventServer *event_server=CEventServer::instance();
  std::list<std::string> events;
 
  try{
    if(this->head_driver!=NULL)
    {
      if(this->motion_sequence!=NULL)
      { 
        delete this->motion_sequence;
        this->motion_sequence=NULL;
        this->head_driver->move(position,velocity,acceleration);
      }
      else
      {
        events.push_back(this->head_driver->get_position_reached_event_id());
        event_server->wait_all(events);
        this->head_driver->move(position,velocity,acceleration);
      }
    }
  }catch(CException &e){
    std::cout << e.what() << std::endl;
    throw;  
  }
}

void TibiDaboHeadDriver::stop(void)
{
  if(this->head_driver!=NULL)
    this->head_driver->stop();
}

void TibiDaboHeadDriver::get_position(std::vector<float> &position)
{
  if(this->head_driver!=NULL)
    this->head_driver->get_position(position);
}

void TibiDaboHeadDriver::get_velocity(std::vector<float> &velocity)
{
  if(this->head_driver!=NULL)
    this->head_driver->get_velocity(velocity);
}

void TibiDaboHeadDriver::set_absolute_motion(bool status)
{
  std::vector<bool> absolute;
  unsigned int i=0;

  if(this->head_driver!=NULL)
  {
    try{
      for(i=0;i<this->head_driver->get_num_motors();i++)
        absolute.push_back(status);
      this->head_driver->set_absolute_motion(absolute);
    }catch(CException &e){
      std::cout << e.what() << std::endl;
      throw;
    }
  }
}

void TibiDaboHeadDriver::set_position_control(bool status)
{
  if(this->head_driver!=NULL)
  {
    try{
      if(status==true)
        this->head_driver->set_position_control();
      else
        this->head_driver->set_velocity_control();
    }catch(CException &e){
      std::cout << e.what() << std::endl;
      throw;
    }
  }
}

// face expressions API
void TibiDaboHeadDriver::set_face_expression(std::string &expression)
{
  if(this->head_driver!=NULL)
  {
    if(expression=="stand_by")
      this->head_driver->stand_by();
    else if(expression=="happy")
      this->head_driver->happy();
    else if(expression=="sad")
      this->head_driver->sad();
    else if(expression=="angry")
      this->head_driver->angry();
    else if(expression=="confused")
      this->head_driver->confused();
    else if(expression=="shy")
      this->head_driver->shy();
    else if(expression=="ashamed")
      this->head_driver->ashamed();
    else if(expression=="speak")
      this->head_driver->speak();
  }
}

void TibiDaboHeadDriver::set_face_expression(int index)
{
  if(this->head_driver!=NULL)
  {
    switch(index)
    {
      case 0: this->head_driver->stand_by();
              break;
      case 1: this->head_driver->happy();
              break;
      case 2: this->head_driver->sad();
              break;
      case 3: this->head_driver->angry();
              break;
      case 4: this->head_driver->confused();
              break;
      case 5: this->head_driver->shy();
              break;
      case 6: this->head_driver->ashamed();
              break;
      case 7: this->head_driver->speak();
              break;
      default: /* no nothing */
               break;
    }
  }
}

void TibiDaboHeadDriver::set_brightness(float brightness)
{
  if(this->head_driver!=NULL)
    this->head_driver->set_brightness(brightness);
}

unsigned int TibiDaboHeadDriver::get_num_config_files(void)
{
  return this->config_files.size();
}

std::string TibiDaboHeadDriver::get_config_file(unsigned int index)
{
  if(index > this->config_files.size())
  {
    /* handle exceptions */
  }
  return this->config_files[index];
}

unsigned int TibiDaboHeadDriver::get_num_motion_seq_files(void)
{
  return this->motion_seq_files.size();
}

std::string TibiDaboHeadDriver::get_motion_seq_file(unsigned int index)
{
  if(index > this->motion_seq_files.size())
  {
    /* handle exceptions */
  }
  return this->motion_seq_files[index];
}

unsigned int TibiDaboHeadDriver::get_num_lights_seq_files(void)
{
  return this->lights_seq_files.size();
}

std::string TibiDaboHeadDriver::get_lights_seq_file(unsigned int index)
{
  if(index > this->lights_seq_files.size())
  {
    /* handle exceptions */
  }
  return this->lights_seq_files[index];
}

int TibiDaboHeadDriver::get_feedback_rate(void)
{
  return this->feedback_rate;
}

TibiDaboHeadDriver::~TibiDaboHeadDriver()
{
  this->stop();
  if(this->motion_sequence!=NULL)
  {
    delete this->motion_sequence;
    this->motion_sequence=NULL;
  } 
  this->stop_lights_sequence();
  if(this->lights_sequence!=NULL)
  {
    delete this->lights_sequence;
    this->lights_sequence=NULL;
  } 
  if(this->head_driver!=NULL)
  {
    delete this->head_driver;
    this->head_driver=NULL;
  }
}
