#include "loquendo_tts_driver.h"

#include "exceptions.h"
#include "loquendo_tts_exceptions.h"

LoquendoTtsDriver::LoquendoTtsDriver(void)
{
  tts = NULL;
  setDriverId("loquendo_tts");
}

bool LoquendoTtsDriver::openDriver(void)
{ 
  try
  {
    this->lock();
      tts = new CLoquendoTTS(config_.speaker);
    this->unlock();
  }
  catch(CException &e)
  {
    ROS_FATAL("'%s'", e.what().c_str());
    this->unlock();

    return false;
  }

  return true;
}

bool LoquendoTtsDriver::closeDriver(void)
{
  delete tts;
  return true;
}

bool LoquendoTtsDriver::startDriver(void)
{
  return true;
}

bool LoquendoTtsDriver::stopDriver(void)
{
  return true;
}

void LoquendoTtsDriver::config_update(const Config& new_cfg, uint32_t level)
{
  this->lock();
  
  try
  {
    // depending on current state
    // update driver with new_cfg data
    switch(this->getState())
    {
      case LoquendoTtsDriver::CLOSED:
        break;

      case LoquendoTtsDriver::OPENED:
        break;

      case LoquendoTtsDriver::RUNNING:
        break;
    }

    // update current speaker voice
    if( tts != NULL )
      tts->setCurrentSpeaker(new_cfg.speaker);
  }
  catch(CException &e)
  {
    ROS_ERROR("'%s'", e.what().c_str());
  }

  // save the current configuration
  this->config_ = new_cfg;

  this->unlock();
}

LoquendoTtsDriver::~LoquendoTtsDriver()
{
  //delete tts;
}

bool LoquendoTtsDriver::playSpeech(const std::string & msg)
{
//   try
//   {
    tts->playSpeech(msg);
    return true;
//   }
//   catch(CLoquendoTTSException &e)
//   {
//     ROS_ERROR("'%s'", e.what().c_str());
//     return false;
//   }
}

bool LoquendoTtsDriver::stopSpeech(void)
{
//   try
//   {
    tts->stopSpeech();
    return true;
//   }
//   catch(CLoquendoTTSException &e)
//   {
//     ROS_ERROR("'%s'", e.what().c_str());
//     return false;
//   }
}

bool LoquendoTtsDriver::pauseSpeech(void)
{
//   try
//   {
    tts->pauseSpeech();
    return true;
//   }
//   catch(CLoquendoTTSException &e)
//   {
//     ROS_ERROR("'%s'", e.what().c_str());
//     return false;
//   }
}

bool LoquendoTtsDriver::resumeSpeech(void)
{
//   try
//   {
    tts->resumeSpeech();
    return true;
//   }
//   catch(CLoquendoTTSException &e)
//   {
//     ROS_ERROR("'%s'", e.what().c_str());
//     return false;
//   }
}

bool LoquendoTtsDriver::isBusy(void)
{
  //{ TO_BE_INIT, AVAILABLE, BUSY, PAUSED };
  if( tts->getCurrentState() != CLoquendoTTS::AVAILABLE)
    return true;
  else
    return false;
}


