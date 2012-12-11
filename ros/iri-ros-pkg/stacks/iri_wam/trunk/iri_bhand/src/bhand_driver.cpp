#include "bhand_driver.h"

using namespace std;

BhandDriver::BhandDriver()
{
  //setDriverId(driver string id);
  this->bhserver_ip = "192.168.101.1";
  this->server_port = 4322;
}

bool BhandDriver::openDriver(void)
{
   string input;
  //setDriverId(driver string id);
  try{
    if(this->state_ != OPENED){
        this->bhand = new BarrettHand(this->bhserver_ip, this->server_port);
        bhand->open();
        this->state_ = OPENED;
        ROS_INFO("bhand opened");
        return true;
    }else{
        ROS_ERROR("WAM was already opened!");
        return false;
    }
  }catch(CException &e){
    ROS_ERROR("%s",e.what().c_str());
    return false;
  }
}

bool BhandDriver::closeDriver(void)
{
  bhand->close();
  ROS_INFO("[APP] Barrett hand closed.");
  this->state_ = CLOSED;
  return true;
}

bool BhandDriver::startDriver(void)
{
  try{
    //gravity and go to position?
    ROS_INFO("Switching to Running");
    this->state_ = RUNNING;
    return true;
  }catch(CException &e){
    ROS_ERROR("%s",e.what().c_str());
    return false;
  }
  return true;
}

bool BhandDriver::stopDriver(void)
{
  return true;
}

void BhandDriver::config_update(const Config& new_cfg, uint32_t level)
{
  this->lock();
  
  // depending on current state
  // update driver with new_cfg data
  switch(this->getState())
  {
    case BhandDriver::CLOSED:
      break;

    case BhandDriver::OPENED:
      break;

    case BhandDriver::RUNNING:
      break;
  }

  // save the current configuration
  this->config_=new_cfg;

  this->unlock();
}

int BhandDriver::rawCommand(string data){
    
    return bhand->rawCommand(data);

}

BhandDriver::~BhandDriver()
{
}
