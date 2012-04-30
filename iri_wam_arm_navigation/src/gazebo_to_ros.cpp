#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>




class GazeboRos
{
	public:
	GazeboRos():
	root_handle_("/"),
	private_handle_("~")
	{
		initPublisher();
		initSubscriber();
	}
	
	~GazeboRos(){}
	
	void initSubscriber()
	{
		gazebo = root_handle_.subscribe("/gazebo/link_states", 1, &GazeboRos::gazeboCB, this);
	}
	
	void initPublisher()
	{
		ros=root_handle_.advertise<geometry_msgs::Pose>("current_pose",5);
	}

	private:
	
	 ros::NodeHandle root_handle_;
	 ros::NodeHandle private_handle_;
	 ros::Subscriber gazebo;
	 ros::Publisher  ros;
	// boost::mutex lock;
	 
	void gazeboCB(const gazebo_msgs::LinkStates::ConstPtr &state)
	{
		//lock.lock();
 /*	std::string tcp="iri_wam::wam_fk/wam7";
		int position;
		findPositionLink(tcp,state->name,position);
	    std::vector<geometry_msgs::Pose> vec=state->pose;
	    geometry_msgs::Pose pos=vec[position];
  	    geometry_msgs::PoseStamped ps;
		ps.pose.position.x=pos.position.x;
		ps.pose.position.y=pos.position.y;
		ps.pose.position.z=pos.position.z;
		ps.pose.orientation.x=pos.orientation.x;
		ps.pose.orientation.y=pos.orientation.y;
		ps.pose.orientation.z=pos.orientation.z;
		ps.pose.orientation.w=pos.orientation.w;
		ps.header.frame_id="/world";
		ps.header.stamp=ros::Time::now();
		ros.publish(ps);*/
		geometry_msgs::Pose pos=state->pose[9];
		ros.publish(pos);
		//lock.unlock();
	}
	void findPositionLink(const std::string s1, const std::vector<std::string> v1, int& pos)
	{	
		pos=0;
		std::vector<std::string>::const_iterator it=v1.begin();
		bool ok=false;
		while(it != v1.end())
		{
  		 if( s1.compare(*it)==0)
			{
				++pos;
				it=v1.end();
				ok=true;
			}
			if(!ok)it++;
		}
	}
};
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "bridge_gazebo");
	GazeboRos g;
	ros::spin();
	return 0;
}

