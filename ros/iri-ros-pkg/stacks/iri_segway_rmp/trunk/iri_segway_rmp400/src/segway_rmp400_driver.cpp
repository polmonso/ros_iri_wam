#include <iridrivers/segway_RMP400_exception.h>
#include <exception>
#include "segway_rmp200_status.h" // used to build ROS status messages
#include "segway_rmp400_driver.h"

SegwayRmp400Driver::SegwayRmp400Driver()
{
    //setDriverId(driver string id);
    try
    {
        segway_ = new CSegwayRMP400();
    }
    catch (CSegwayRMP400Exception & e)
    {
        ROS_FATAL("'%s'",e.what().c_str());
    }
}

bool
SegwayRmp400Driver::openDriver(void)
{
    try
    {
        segway_->start();
        segway_->reset_integrators();
    }
    catch (CException & e)
    {
        ROS_ERROR("'%s'", e.what().c_str());
        return false;
    }
    catch (std::exception & e)
    {
        ROS_ERROR("Unexpected exception '%s': ",e.what());
        return false;
    }

    return true;
}

bool
SegwayRmp400Driver::closeDriver(void)
{
    try
    {
        segway_->stop();
    }
    catch (CException & e)
    {
        ROS_ERROR("'%s'", e.what().c_str());
        return false;
    }

    return true;
}

bool
SegwayRmp400Driver::startDriver(void)
{
    return true;
}

bool
SegwayRmp400Driver::stopDriver(void)
{
    return true;
}

void
SegwayRmp400Driver::config_update(const Config& new_cfg, uint32_t level)
{
  this->lock();

  //update driver with new_cfg data

  // save the current configuration
  this->config_=new_cfg;

  this->unlock();
}

iri_segway_rmp_msgs::SegwayRMP400Status
SegwayRmp400Driver::get_status(void)
{
    iri_segway_rmp_msgs::SegwayRMP400Status ROS_status;
    TSegwayRMP400Status segway_status = segway_->get_status();

    ROS_status.rmp200[0] = segway_rmp200_node::build_ROS_status_msg(segway_status.rmp200[0]);
    ROS_status.rmp200[1] = segway_rmp200_node::build_ROS_status_msg(segway_status.rmp200[1]);

    return ROS_status;
}

void
SegwayRmp400Driver::move(const geometry_msgs::Twist::ConstPtr & msg)
{
    lock();

    // For movement we define axis X as only possible segway linear movement
    double vT = msg->linear.x;
    double vR = msg->angular.z;

    ROS_DEBUG("Movement sent to platform. vT = '%f' vR = '%f'", vT, vR);
    segway_->move(vT,vR);

    unlock();
}

SegwayRmp400Driver::~SegwayRmp400Driver()
{
    delete segway_;
}
