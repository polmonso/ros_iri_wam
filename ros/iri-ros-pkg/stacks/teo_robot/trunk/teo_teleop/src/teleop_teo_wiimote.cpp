#include <ros/ros.h>
#include <iri_wiimote_teleop/wiimote_teleop_base.h>
#include <iri_hokuyo_laser3d/Get3DScan.h>

class TeleopTeo : public WiimoteTeleop
{
    public:
        TeleopTeo();

    protected:
        virtual void useExtraButton(const unsigned int & index);
};

TeleopTeo::TeleopTeo()
{ }

void TeleopTeo::useExtraButton(const unsigned int & index)
{
  ROS_DEBUG("TEO wiimote use extra Button");
  iri_hokuyo_laser3d::Get3DScan get_pointcloud;
  switch(index)
  {
    case wiimote::State::MSG_BTN_1:
      ROS_DEBUG("Button 1!");
      break;
    case wiimote::State::MSG_BTN_2:
      ROS_DEBUG("Button 2!");
      break;
    case wiimote::State::MSG_BTN_B:
      ROS_DEBUG("Button B!");
      break;
    case wiimote::State::MSG_BTN_PLUS:
      get_pointcloud.request.request = 1;
      if(ros::service::exists("/teo/h3d/get_3d_scan",true))
      {
        if(ros::service::call("/teo/h3d/get_3d_scan", get_pointcloud))
          ROS_DEBUG("get_3d_scan Service Called");
        else
          ROS_WARN("get_3d_scan Service NOT Called");
      }else{
        ROS_WARN("get_3d_scan service doesn't exist!");
      }
      ROS_DEBUG("Button +! 3D SCAN");
      break;
    case wiimote::State::MSG_BTN_MINUS:
      ROS_DEBUG("Button -!");
      break;
    case wiimote::State::MSG_BTN_HOME:
      ROS_DEBUG("Button Caseta!");
      break;
  }
}

int main(int argc,char *argv[])
{
    return iri_base_joystick::main<TeleopTeo>(argc, argv, "teo_teleop_node");
}
