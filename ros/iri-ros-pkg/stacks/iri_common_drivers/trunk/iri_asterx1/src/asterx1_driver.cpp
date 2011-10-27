#include "asterx1_driver.h"

Asterx1Driver::Asterx1Driver()
{
	//setDriverId(driver string id);
  
	gpsDevice = new CasteRx1("/dev/ttyACM0", MSEC500);
}

bool Asterx1Driver::openDriver(void)
{
  //setDriverId(driver string id);

	if ( gpsDevice->openDevice(5) == BASIC_SUCCESS )
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool Asterx1Driver::closeDriver(void)
{
	if ( gpsDevice->closeDevice() == BASIC_SUCCESS )
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool Asterx1Driver::startDriver(void)
{
	if ( gpsDevice->startAcquisition() == BASIC_SUCCESS )
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool Asterx1Driver::stopDriver(void)
{
	if ( gpsDevice->stopAcquisition() == BASIC_SUCCESS )
	{
		return true;
	}
	else
	{
		return false;
	}
}

void Asterx1Driver::config_update(const Config& new_cfg, uint32_t level)
{
	this->lock();
  
  //update driver with new_cfg data
	try
	{
		if( this->getState() == Asterx1Driver::CLOSED )
		{
			this->gpsDevice->setPortName(new_cfg.serialPortName);
		}
		if( (this->getState() != Asterx1Driver::RUNNING) )
		{
			this->gpsDevice->setAcquisitionRate(new_cfg.acquisitionRate);
		}
	}
	catch(CException &e)
	{
		ROS_ERROR("'%s'", e.what().c_str());
	}

  // save the current configuration
	this->config_=new_cfg;

	this->unlock();
}

Asterx1Driver::~Asterx1Driver()
{
	delete gpsDevice;
}

void Asterx1Driver::setMapOrigin(double mapLat0, double mapLon0, double mapAlt0, double mapAlpha0)
{
	gpsDevice->setMapOrigin(mapLat0, mapLon0, mapAlt0, mapAlpha0);
}

void Asterx1Driver::printTM()
{
	gpsDevice->printTM();
}

int Asterx1Driver::readDataFromDevice()
{
	return gpsDevice->readDataFromDevice();
}

unsigned int Asterx1Driver::getStatus(){ return gpsDevice->getStatus();}
double Asterx1Driver::getTimeStamp(){ return gpsDevice->getTimeStamp();}
unsigned short int Asterx1Driver::getWnc(){ return gpsDevice->getWnc();}
unsigned int Asterx1Driver::getTow(){ return gpsDevice->getTow();}
unsigned int Asterx1Driver::getNumSatellites(){ return gpsDevice->getNumSatellites();}
float Asterx1Driver::getPDOP(){ return gpsDevice->getPDOP();}
float Asterx1Driver::getTDOP(){ return gpsDevice->getTDOP();}
float Asterx1Driver::getHDOP(){ return gpsDevice->getHDOP();}
float Asterx1Driver::getVDOP(){ return gpsDevice->getVDOP();}
float Asterx1Driver::getUndulation(){ return gpsDevice->getUndulation();}
unsigned short int Asterx1Driver::getPVTerror(){ return gpsDevice->getPVTerror();}
double Asterx1Driver::getLat(bool units){ return gpsDevice->getLat(units);} 
double Asterx1Driver::getLon(bool units){ return gpsDevice->getLon(units);} 
double Asterx1Driver::getAlt(){ return gpsDevice->getAlt();}
double Asterx1Driver::getXWgs(){ return gpsDevice->getXWgs();}
double Asterx1Driver::getYWgs(){ return gpsDevice->getYWgs();}
double Asterx1Driver::getZWgs(){ return gpsDevice->getZWgs();}
double Asterx1Driver::getXMap(){ return gpsDevice->getXMap();}
double Asterx1Driver::getYMap(){ return gpsDevice->getYMap();}
double Asterx1Driver::getZMap(){ return gpsDevice->getZMap();}
double Asterx1Driver::getVxWgs(){ return gpsDevice->getVxWgs();}
double Asterx1Driver::getVyWgs(){ return gpsDevice->getVyWgs();}
double Asterx1Driver::getVzWgs(){ return gpsDevice->getVzWgs();}
double Asterx1Driver::getVxMap(){ return gpsDevice->getVxMap();}
double Asterx1Driver::getVyMap(){ return gpsDevice->getVyMap();}
double Asterx1Driver::getVzMap(){ return gpsDevice->getVzMap();}
double Asterx1Driver::getCxxWgs(){ return gpsDevice->getCxxWgs();}
double Asterx1Driver::getCyyWgs(){ return gpsDevice->getCyyWgs();}
double Asterx1Driver::getCzzWgs(){ return gpsDevice->getCzzWgs();}
double Asterx1Driver::getCxyWgs(){ return gpsDevice->getCxyWgs();}
double Asterx1Driver::getCxxMap(){ return gpsDevice->getCxxMap();}
double Asterx1Driver::getCyyMap(){ return gpsDevice->getCyyMap();}
double Asterx1Driver::getCzzMap(){ return gpsDevice->getCzzMap();}
double Asterx1Driver::getCxyMap(){ return gpsDevice->getCxyMap();}
double Asterx1Driver::getCxxEnu(){ return gpsDevice->getCxxEnu();}
double Asterx1Driver::getCyyEnu(){ return gpsDevice->getCyyEnu();}
double Asterx1Driver::getCzzEnu(){ return gpsDevice->getCzzEnu();}
double Asterx1Driver::getCxyEnu(){ return gpsDevice->getCxyEnu();}
double Asterx1Driver::getCxzEnu(){ return gpsDevice->getCxzEnu();}
double Asterx1Driver::getCyzEnu(){ return gpsDevice->getCyzEnu();}
