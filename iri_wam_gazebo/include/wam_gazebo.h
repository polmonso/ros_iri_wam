#ifndef WAM_GAZEBO_HH
#define WAM_GAZEBO_HH

#include <map>
#include <gazebo/gazebo.h>
#include <gazebo/Param.hh>
#include <gazebo/Controller.hh>
#include <gazebo/Model.hh>

#include <gazebo/Pose3d.hh>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>

#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/Pose3d.hh>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <actionlib/server/simple_action_server.h>
#include <tinyxml.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include "iri_wam_common_msgs/GeneralState.h"
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
typedef  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle GoalHandle;	
namespace gazebo
{
	class Body;
    class Joint;
    class Entity;

    class WAMGazebo :
        public Controller
    {

        public:
          WAMGazebo(Entity * parent);
          virtual ~WAMGazebo();

        protected:
          virtual void LoadChild(XMLConfigNode *node);
          virtual void InitChild();
          virtual void ResetChild();
          virtual void UpdateChild();
          virtual void FiniChild();

        private:

          libgazebo::PositionIface *pos_iface_;
          Model * parent_;

		  std::vector<Joint*> joints_;	  
		  std::vector<Body*>  bodys_;	  
		  std::vector<double> joints_positions;
		  		  
		  std::vector<Pose3d> init_positions;
          std::vector<Pose3d> actual_positions;
          std::vector<Vector3> anchors;
          std::vector<std::string> names_joints;
          std::vector<std::string> names_bodys;
          
          Pose3d zeroBase;
          // ROS STUFF
          ros::NodeHandle* rosnode_;                    
          ros::Publisher state_publisher;
          ros::Publisher command;
        //  ros::Subscriber state;
          // Custom Callback Queue
          ros::CallbackQueue queue_;
          boost::thread* callback_queue_thread_;          

          std::string robotNamespace;
          
          ros::NodeHandle  nodo_;
          ros::NodeHandle  global_;
          actionlib::ActionServer <control_msgs::FollowJointTrajectoryAction > server;
          ros::ServiceServer service_;
          bool alive_;
          bool inMoving;

          boost::mutex lock;
          
          ParamT<std::string> * robotNamespaceP;

          /** BODYS  **/
                    
          ParamT<std::string> * body_0;          
          ParamT<std::string> * body_1;
          ParamT<std::string> * body_2;
          ParamT<std::string> * body_3;
          ParamT<std::string> * body_4;
          ParamT<std::string> * body_5;
          ParamT<std::string> * body_6;
          ParamT<std::string> * body_7;
          ParamT<std::string> * body_8;
          /** JOINTS **/
          ParamT<std::string> * joint_0;
          ParamT<std::string> * joint_1;
          ParamT<std::string> * joint_2;
          ParamT<std::string> * joint_3;
          ParamT<std::string> * joint_4;
          ParamT<std::string> * joint_5;
          ParamT<std::string> * joint_6;
          ParamT<std::string> * joint_7;

         void QueueThread();
		 void goalCB(GoalHandle gh);
		 void initTopics();
		 void copyVector(const std::vector<Pose3d>& a,std::vector<Pose3d>& b);
		 void loadPose(const std::vector<Pose3d>& pose);
         void getState(std::vector<double>& jnt);
         void sendState(const std::vector<double>& jnt);
		 void getPoses(std::vector<Pose3d>& poses);
		 void pointToMove(trajectory_msgs::JointTrajectoryPoint& jtp,std::vector<std::string> name);
		 bool findJointPosition(double &position, std::string name, std::vector<std::string> joint_names, std::vector<double> joint_positions);
		 bool findJointVelocity(double &velocity, std::string name, std::vector<std::string> joint_names, std::vector<double> joint_velocities);
		 bool findJointAccel(double &accel, std::string name, std::vector<std::string> joint_names, std::vector<double> joint_accelarations);
		 Body* getParentBody(Joint* joint);
		 Body* getChildBody(Joint* joint);
		 void rotateBodyAndChildren(Body* body1, Vector3 anchor,Vector3 axis, double dangle, bool update_children); 
		 void getAllChildrenBodies(std::vector<Body*> &bodies, Model* model, Body* body);
		 void getAllParentBodies(std::vector<Body*> &bodies, Model* model, Body* body, Body* orig_parent_body);
		 bool inBodies(Body* body,std::vector<Body*> bodies);
		 void getPosition(Joint* j1,double& pos);
		 void publishFeed(int st);
		 void update(int i);
		 bool GenerealStateCB(iri_wam_common_msgs::GeneralState::Request& request,iri_wam_common_msgs::GeneralState::Response& response);
		  actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>* traj_client_;
    };
}

#endif
