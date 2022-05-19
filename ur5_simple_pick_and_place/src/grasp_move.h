#ifndef _DRIVER_H
#define _DRIVER_H
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <vector>
using namespace std;
using namespace moveit::planning_interface;

class GraspMove
{
   private:
    MoveGroupInterface* i_arm;
    MoveGroupInterface* i_gripper;
    PlanningSceneInterface * i_scene;
    
    void PickPlaceOne(vector<float> &size, vector<float> &position);   
    void GoHome(MoveGroupInterface::Plan &my_plan);

   public:
    GraspMove();
    ~GraspMove();
    void PickPlace();
};
#endif