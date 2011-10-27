/**
 * @file /include/tibi_dabo_sequence_editor/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef tibi_dabo_sequence_editor_QNODE_HPP_
#define tibi_dabo_sequence_editor_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
// [publisher subscriber headers]
#include <sensor_msgs/JointState.h>

// [action server client headers]
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

// GUI
#include <string>
#include <QThread>
#include <QStringListModel>

#include "mutex.h"
#include "eventserver.h"
#include "motion_sequence.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tibi_dabo_sequence_editor {

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> SeqClient;

typedef struct
{
  SeqClient *client;
  bool succeeded;
  bool active;
  bool loaded;
  float percentage;
}TSeqClient;

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread 
{
  Q_OBJECT
  public:
    QNode(int argc, char** argv );
    ~QNode();
    void run();
    void get_motion_feedback(std::vector<float> &position, std::vector<float> &velocity);
    std::vector<std::string> get_config_files(void);
    bool is_connected(void);
    std::string get_new_feedback_event_id(void);
    std::string get_action_feedback_event_id(void);
    void execute_sequence(std::vector<TMotionStep> &seq);

  signals:
    void rosShutdown();

  private:
    // [subscriber attributes]
    ros::Subscriber motion_feedback_subscriber_;
    void motion_feedback_callback(const sensor_msgs::JointState::ConstPtr& msg);
    std::vector<float> position;
    std::vector<float> velocity;
    CMutex motion_feedback_mutex_;

    // motion sequence
    TSeqClient motion_;
    void motionMakeActionRequest(const control_msgs::FollowJointTrajectoryGoal & motion_goal);
    void motionDone(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result);
    void motionActive(void);
    void motionFeedback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback);

    // parameter values
    std::string node_name;
    std::vector<std::string> config_files;
    CEventServer *event_server;
    std::string new_feedback_event_id;
    std::string action_feedback_event_id;
};

}  // namespace tibi_dabo_sequence_editor

#endif /* tibi_dabo_sequence_editor_QNODE_HPP_ */
