#include "iri_base_driver/iri_base_driver.h"

namespace iri_base_driver
{

IriBaseDriver::IriBaseDriver() : driver_id_("none")
{
}

void IriBaseDriver::lock(void)
{
  this->access_.enter();
}

void IriBaseDriver::unlock(void)
{
  this->access_.exit();
}

bool IriBaseDriver::try_enter(void)
{
  return this->access_.try_enter();
}

void IriBaseDriver::setDriverId(const std::string & id)
{
  driver_id_ = id;
}

void IriBaseDriver::doOpen(void)
{
  if(openDriver()) this->state_ = OPENED;
}

void IriBaseDriver::doClose(void)
{
  this->preCloseHook();
  if(closeDriver()) this->state_ = CLOSED;
}

void IriBaseDriver::doStart(void)
{
  if(startDriver()) this->state_ = RUNNING;
}

void IriBaseDriver::doStop(void)
{
  if(stopDriver()) this->state_ = OPENED;
}

std::string IriBaseDriver::getID(void)
{
  return driver_id_;
}

void IriBaseDriver::setPreCloseHook(hookFunction f)
{
  preCloseHook = f;
}

IriBaseDriver::~IriBaseDriver()
{
}

}
