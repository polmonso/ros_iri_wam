#include "segway_rmp400_driver_node.h"

SegwayRmp400DriverNode::SegwayRmp400DriverNode(ros::NodeHandle &nh) :
    iri_base_driver::IriBaseNodeDriver<SegwayRmp400Driver>(nh),
    tf_broadcaster_()
{
    std::string vel_topic_name;
    std::string status_topic_name;

    // Publish in Segway status topic
    private_node_handle_.param<std::string>("status_topic", status_topic_name, "status");
    status_publisher_  = public_node_handle_.advertise<iri_segway_rmp_msgs::SegwayRMP400Status>
                                                                      (status_topic_name, 100);
    ROS_DEBUG("Publishing status topic at: %s'", status_topic_name.c_str());

    // Receiving vel command in CMD_VEL topic
    private_node_handle_.param<std::string>("cmd_vel", vel_topic_name, "cmd_vel"); 
    cmd_vel_subscriber_ = public_node_handle_.subscribe(vel_topic_name, 100,
                                   & SegwayRmp400DriverNode::cmd_vel_callback, this);
    ROS_DEBUG("Subscribing to cmd_vel_callback topic at: '%s'", vel_topic_name.c_str());
}

void
SegwayRmp400DriverNode::mainNodeThread(void)
{
    update_status();
    status_publisher_.publish(segway_status_msg_);
}

void
SegwayRmp400DriverNode::update_status(void)
{
    segway_status_msg_ = driver_.get_status();
}

void
SegwayRmp400DriverNode::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr & msg)
{
    if (! driver_.isRunning()) {
        ROS_ERROR("Impossible to move, driver is not running");
        return;
    }

    try
    {
        driver_.move(msg);
    }
    catch (CException & e)
    {
        driver_.closeDriver();
        ROS_FATAL("Error while moving via cmd_vel_callback: '%s'", e.what().c_str());
    }
}

void
SegwayRmp400DriverNode::postNodeOpenHook(void)
{
}

void
SegwayRmp400DriverNode::addNodeDiagnostics(void)
{
}

void
SegwayRmp400DriverNode::addNodeOpenedTests(void)
{
}

void
SegwayRmp400DriverNode::addNodeStoppedTests(void)
{
}

void
SegwayRmp400DriverNode::addNodeRunningTests(void)
{
}

void
SegwayRmp400DriverNode::reconfigureNodeHook(int level)
{
}

SegwayRmp400DriverNode::~SegwayRmp400DriverNode()
{
  //[free dynamic memory]
}

/* main function */
int main(int argc,char *argv[])
{
  return driver_base::main<SegwayRmp400DriverNode>(argc,argv,"segway_rmp400_driver_node");
}
