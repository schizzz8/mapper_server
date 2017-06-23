#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "mapper_server/MapperAction.h"
#include <boost/thread.hpp>


class MapperClient {
public:
    MapperClient():_start(1),_stop(2),_ac("mapper_server") {

        nh_.param("button_start", _start, _start);
        nh_.param("button_stop", _stop, _stop);

        _joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &MapperClient::joyCallback, this);

        ROS_INFO("Waiting for action server to start.");
        _ac.waitForServer();
        ROS_INFO("Action server started!");
    }

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){

        if(joy->buttons[_start] == 1){
            mapper_server::MapperGoal goal;
            goal.trigger = "go";
            _ac.sendGoal(goal);
            ROS_INFO("Sending goal.");
        }

        if(joy->buttons[_stop]){
            _ac.cancelGoal();
            ROS_INFO("Canceling goal.");
        }
    }

    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<mapper_server::MapperAction> _ac;
    int _start, _stop;
    ros::Subscriber _joy_sub;

};

int main (int argc, char **argv) {
    ros::init(argc, argv, "mapper_client");

    MapperClient client;

    ros::spin();

    return 0;
}
