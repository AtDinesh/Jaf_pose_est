#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <riddle/FindPersonAction.h>


typedef actionlib::SimpleActionClient<riddle::FindPersonAction> Client;

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const riddle::FindPersonResultConstPtr& result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("Answer: [ %f  %f  %f ] ",
                result->position[0],
                result->position[1],
                result->position[2]);
    ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
    ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const riddle::FindPersonFeedbackConstPtr& feedback)
{
    ROS_INFO("Got Feedback [%d] \n", feedback->currentCnt);
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "test_find_person_callback");

    // Create the action client
    Client ac("find_person", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");

    // Send Goal
    riddle::FindPersonGoal goal;
    goal.requiredCnt = 5;
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    ros::spin();
    return 0;
}


//int main (int argc, char **argv)
//{
//  ros::init(argc, argv, "test_fibonacci");

//  // create the action client
//  // true causes the client to spin its own thread
//  actionlib::SimpleActionClient<riddle::FindPersonAction> ac("find_person", true);

//  ROS_INFO("Waiting for action server to start.");
//  // wait for the action server to start
//  ac.waitForServer(); //will wait for infinite time

//  ROS_INFO("Action server started, sending goal.");
//  // send a goal to the action
//  riddle::FindPersonGoal goal;
//  goal.requiredCnt = 6;
//  ac.sendGoal(goal);

//  //wait for the action to return
//  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

//  if (finished_before_timeout)
//  {
//    actionlib::SimpleClientGoalState state = ac.getState();
//    ROS_INFO("Action finished: %s",state.toString().c_str());
//  }
//  else
//    ROS_INFO("Action did not finish before the time out.");

//  //exit
//  return 0;
//}
