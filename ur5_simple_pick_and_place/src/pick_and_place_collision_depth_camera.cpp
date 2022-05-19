/*
a1: Added step 10. Move to home position
b1: Add function applyCollisionObject1, applyCollisionObject2
a2: By #define B2, the robot can sucessuflly two objects seperately.
a3: Add class grasp_move. The robot can move two cube sequently.
*/

#include "grasp_move.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle n;

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
    ros::AsyncSpinner spinner(1);
    spinner.start();
    GraspMove grasp_move;
    grasp_move.PickPlace();
    ros::shutdown();
    return 0;
}
