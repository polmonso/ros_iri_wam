#include "segway_rmp400_odom_3Dexpmaps.h"

#define DEBUG_JAC 1

SegwayRmp400Odom::SegwayRmp400Odom()
{
  accum.translation.x=0.0;
  accum.translation.y=0.0;
  accum.translation.z=0.0;
  accum.rotation.x=0.0;
  accum.rotation.y=0.0;
  accum.rotation.z=0.0;
  accum.rotation.w=1.0;

  H.setIdentity();
  if (DEBUG_JAC) {
    ROS_INFO("odo3d Hinit: %f %f %f %f", H(0,0), H(0,1), H(0,2), H(0,3));
    ROS_INFO("         %f %f %f %f", H(1,0), H(1,1), H(1,2), H(1,3));
    ROS_INFO("         %f %f %f %f", H(2,0), H(2,1), H(2,2), H(2,3));
    ROS_INFO("         %f %f %f %f", H(3,0), H(3,1), H(3,2), H(3,3));
  }
}

SegwayRmp400Odom::~SegwayRmp400Odom()
{ }

void
SegwayRmp400Odom::config_update(const Config& new_cfg, uint32_t level)
{
  config_ = new_cfg;
}

//robot frame
  //  x:fwd, y:left-side, z:up
void
SegwayRmp400Odom::computeOdometry(const iri_segway_rmp_msgs::SegwayRMP400Status & msg)
{
  current_time_ = ros::Time::now();
  // calculate the delta T
  dt = (current_time_ - last_time_).toSec();
  ROS_INFO("odo3D dt: %f", dt);

  // only if dt is less than one second...
  if(dt<1.0) {

    // 1. Get rotational velocity in rads
    vroll  =   ((msg.rmp200[0].roll_rate  + msg.rmp200[1].roll_rate ) / 2);// *PI/180;
    vpitch =   ((msg.rmp200[0].pitch_rate + msg.rmp200[1].pitch_rate) / 2);// *PI/180;
    vyaw   =  -((msg.rmp200[0].yaw_rate   + msg.rmp200[1].yaw_rate  ) / 2);// *PI/180;
    if (DEBUG_JAC)  ROS_INFO("odo3D vyaw  : %f", vyaw);
    if (DEBUG_JAC)  ROS_INFO("odo3D vpitch: %f", vpitch);
    if (DEBUG_JAC)  ROS_INFO("odo3D vroll : %f", vroll);

    // 2. Get current translational velocity 
    v_left_wheels  = (msg.rmp200[0].left_wheel_velocity +
                           msg.rmp200[1].left_wheel_velocity) / 2;
    v_right_wheels = (msg.rmp200[0].right_wheel_velocity +
                           msg.rmp200[1].right_wheel_velocity) / 2;
    if (DEBUG_JAC)  ROS_INFO("odo3D vleft : %f", v_left_wheels);
    if (DEBUG_JAC)  ROS_INFO("odo3D vright : %f", v_right_wheels);

    // 3. Form the rotational velocity vector in local coords
    w << vroll, vpitch, vyaw;
    if (DEBUG_JAC)  ROS_INFO("odo3D w: %f %f %f", w(0), w(1), w(2));

    // 4. Compute the local rotation
    wt = w*dt;
    if (DEBUG_JAC)  ROS_INFO("odo3D wt: %f %f %f", wt(0), wt(1), wt(2));

    // 5. Compute norm of rotation vector
    wt_norm = wt.norm();
    if (DEBUG_JAC) ROS_INFO("odo3D wtnorm: %f", wt_norm);

    // 6. Compute the unit norm rotation axis
    ww = wt/wt_norm;
    if (DEBUG_JAC) ROS_INFO("odo3D ww: %f %f %f", ww(0), ww(1), ww(2));

    // 7. Build the skew symmetric form of ww
    wt_hat << 0.0, -ww(2), ww(1), ww(2), 0.0, -ww(0), -ww(1), ww(0), 0.0;

    // 8. Build the local velocity vector

    v << ((v_left_wheels + v_right_wheels)/2), 0.0, 0.0;
    if (DEBUG_JAC)  ROS_INFO("odo3D v: %f %f %f", v(0), v(1), v(2));

    // 9. Compute exponential map of twist

    epsilon = 1e-9;

    I.setIdentity();


    if (wt_norm < epsilon) {
      R.setIdentity();    
      t =v*dt;
      if (DEBUG_JAC)  ROS_INFO("odo3D: warning rotation < epsilon!");
    }
    else {
      R = I + wt_hat * sin(wt_norm) + wt_hat*wt_hat*(1-cos(wt_norm));
      // t = (I-R)*wt_hat*v  +  ww.transpose()*v*ww*wt_norm;
      //t = (I-R)*wt_hat*v  +  (ww(0)*v(0)+ww(1)*v(1)+ww(2)*v(2))*ww*wt_norm;
      ta=(I-R)*wt_hat*v*dt/wt_norm ;
      tb= (ww(0)*v(0)+ww(1)*v(1)+ww(2)*v(2))*ww*dt;
     t = ta+tb;
     if (DEBUG_JAC)  ROS_INFO("odo3D: warning rotation > epsilon: t = %f %f %f + %f %f %f",ta(0),ta(1),ta(2),tb(0),tb(1),tb(2));

 }



  if (DEBUG_JAC) {
    ROS_INFO("odo3d R: %f %f %f", R(0,0), R(0,1), R(0,2));
    ROS_INFO("         %f %f %f", R(1,0), R(1,1), R(1,2));
    ROS_INFO("         %f %f %f", R(2,0), R(2,1), R(2,2));
    ROS_INFO("odo3D t: %f %f %f", t(0), t(1), t(2));
  }

//  if (DEBUG_JAC)   std::cout << "t\n " << t <<std::endl;

  g.setIdentity();
  g.block<3,3>(0,0) = R;
  g.block<3,1>(0,3) = t;

  H = H*g;

  if (DEBUG_JAC) {
    ROS_INFO("odo3d H: %f %f %f", H(0,0), H(0,1), H(0,2));
    ROS_INFO("         %f %f %f", H(1,0), H(1,1), H(1,2));
    ROS_INFO("         %f %f %f", H(2,0), H(2,1), H(2,2));
  }

//  if (DEBUG_JAC)   std::cout << "H\n " << H <<std::endl;

  d(0) = 1 + H(0,0) - H(1,1) - H(2,2);
  d(1) = 1 - H(0,0) + H(1,1) - H(2,2);
  d(2) = 1 - H(0,0) - H(1,1) + H(2,2);
  d(3) = 1 + H(0,0) + H(1,1) + H(2,2);

  d.maxCoeff(&i);

  switch (i) {
    case 0:
        q(0) = sqrt(d(0))/2;
        q(1) = (H(1,0) + H(0,1)) / (4 * q(0));
        q(2) = (H(2,0) + H(0,2)) / (4 * q(0));
        q(3) = (H(2,1) - H(1,2)) / (4 * q(0));
    break;
    case 1:
        q(1) = sqrt(d(1))/2;
        q(0) = (H(1,0) + H(0,1)) / (4 * q(1));
        q(2) = (H(2,1) + H(1,2)) / (4 * q(1));
        q(3) = (H(0,2) - H(2,0)) / (4 * q(1));
    break;
    case 2:
        q(2) = sqrt(d(2))/2;
        q(0) = (H(2,0) + H(0,2)) / (4 * q(2));
        q(1) = (H(2,1) + H(1,2)) / (4 * q(2));
        q(3) = (H(1,0) - H(0,1)) / (4 * q(2));
    break;
    case 3:
        q(3)= sqrt(d(3))/2;
        q(0) = (H(2,1) - H(1,2)) / (4 * q(3));
        q(1) = (H(0,2) - H(2,0)) / (4 * q(3));
        q(2) = (H(1,0) - H(0,1)) / (4 * q(3));
    break;
    default:
        ROS_INFO("Error computing quaternion");
  }
  
 if (DEBUG_JAC) ROS_INFO("odo3D q: %f %f %f %f", q(0), q(1), q(2), q(3));


  //update angles and position

  accum.translation.x = H(0,3);
  accum.translation.y = H(1,3);
  accum.translation.z = H(2,3);
  accum.rotation.x = q(0);
  accum.rotation.y = q(1);
  accum.rotation.z = q(2);
  accum.rotation.w = q(3);

  transform_.translation =   accum.translation;
  transform_.rotation       = accum.rotation;

  pose_.pose.position.x = accum.translation.x;
  pose_.pose.position.y = accum.translation.y;
  pose_.pose.position.z  = accum.translation.z;
  pose_.pose.orientation = accum.rotation;
  pose_.covariance[0]  =
  pose_.covariance[7]  =
  pose_.covariance[14] =
  pose_.covariance[21] =
  pose_.covariance[28] =
  pose_.covariance[35] = 0.5;

  // Fill the twist with the current translational and rotational velocities
  // in the child coordinate frame (local)

  twist_.twist.linear.x  = vT;
  twist_.twist.linear.y  = 0.0;
  twist_.twist.linear.z  = 0.0;
  twist_.twist.angular.x = vroll;
  twist_.twist.angular.y = vpitch;
  twist_.twist.angular.z = vyaw;
  
  // Add the covariances to use with EKF
  // for x and y velocity, Error ~= +-10%
  twist_.covariance[0]  = pow(0.1*vT,2);
  twist_.covariance[7]  = pow(0.1*vT,2);
  twist_.covariance[14] =
  twist_.covariance[21] =
  twist_.covariance[28] = 1;
  // for theta velocity, Error ~= +-50%
  twist_.covariance[35] = pow(0.5*vyaw,2);

}
  //update last time
  last_time_ = current_time_;

}

int main(int argc, char *argv[])
{
  return algorithm_base::main< GenericOdometry<SegwayRmp400Odom, iri_segway_rmp_msgs::SegwayRMP400Status> >(argc, argv, "segway_rmp400_odom");
}
