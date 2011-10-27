/* vim: set sw=4 sts=4 et foldmethod=syntax : */

#include <ros/ros.h>
#include <joy/Joy.h>
#include <geometry_msgs/Twist.h>
#include <iri_hokuyo_laser3d/Get3DScan.h>

class TeleopTeo
{
    public:
        TeleopTeo();

    private:
        void joyCallback(const joy::Joy::ConstPtr& joy);
        void axisCallback(std::vector<float> axes);
        static bool check_stop_button_callback(const std::vector<int> buttons);
        static bool check_movement_axes_callback(const std::vector<float> axes);

        void generate_movement_from_axes(std::vector<float> axes);
        void generate_movement_stop();

        geometry_msgs::Twist generate_movement_twist() const;

        /* geometry_msgs::Twist transform_to_twist(const joy::Joy & joy); */
        const static unsigned int stop_button_ = 5;
        const static int y_axis_id_ = 1;
        const static int x_axis_id_ = 0;

        float vT_;
        float vR_;

        ros::NodeHandle nh_;
        int linear_, angular_;
        double l_scale_, a_scale_;
        ros::Publisher vel_pub_;
        ros::Subscriber joy_sub_;
};

TeleopTeo::TeleopTeo() :
    vT_(0.0),
    vR_(0.0)
{
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/teo/segway/cmd_vel", 1);
    joy_sub_ = nh_.subscribe<joy::Joy>("joy", 10, &TeleopTeo::joyCallback, this);
}

void
TeleopTeo::generate_movement_stop()
{
    vT_ = 0.0;
    vR_ = 0.0;
}

geometry_msgs::Twist
TeleopTeo::generate_movement_twist() const
{
    geometry_msgs::Twist * msg = new geometry_msgs::Twist;
    msg->linear.x  = vT_;
    msg->angular.z = vR_;
    ROS_DEBUG("Send velocity: vT '%f' vR '%f'", vT_, vR_);

    return * msg;
}

void
TeleopTeo::generate_movement_from_axes(std::vector<float> axes)
{
    // Check for the X axis
    if (axes[x_axis_id_] > 0.0) {
        ROS_DEBUG("RIGHT");
        vR_ -= 0.1;
    }

    if (axes[x_axis_id_] < 0.0) {
        ROS_DEBUG("LEFT");
        vR_ += 0.1;
    }

    // Check for the Y axis
    if (axes[y_axis_id_] > 0.0) {
        ROS_DEBUG("UP");
        vT_ += 0.1;
    }

    if (axes[y_axis_id_] < 0.0) {
        ROS_DEBUG("DOWN");
        vT_ -= 0.1;
    }
}

bool
TeleopTeo::check_stop_button_callback(const std::vector<int> buttons)
{
    if (buttons.size() == 0)
        return false;

    for (unsigned int i = 1; i < buttons.size(); i++)
        if (buttons[i] > 0){
            if (i == stop_button_) {
                ROS_DEBUG("Emergency stop button pressed");
                return true;
            }
            if( i == 4)
            {
              iri_hokuyo_laser3d::Get3DScan get_pointcloud;
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
            }
        }

    return false;
}

/* this should be in the joystick class */
bool
TeleopTeo::check_movement_axes_callback(const std::vector<float> axes)
{
    if (axes.size() == 0)
        return false;

    // Not enough only watching size in axes
    for (unsigned int i =0; i < axes.size(); i++)
        if (axes[i] != 0.0)
            return true;

    return false;
}

void
TeleopTeo::joyCallback(const joy::Joy::ConstPtr & joy)
{
    geometry_msgs::Twist msg;

    // TODO: improve iri joy with methods to check if there is
    // any data in axes/buttons or are empty.
    bool axes_mov    = check_movement_axes_callback(joy->axes);
    bool buttons_mov = check_stop_button_callback(joy->buttons);

    if ((! axes_mov) && (! buttons_mov))
        return;

    if (axes_mov)
        generate_movement_from_axes(joy->axes);

    if (buttons_mov)
        generate_movement_stop();

    msg = generate_movement_twist();
    vel_pub_.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_teo");
    TeleopTeo teleop_teo;

    ros::spin();
}
