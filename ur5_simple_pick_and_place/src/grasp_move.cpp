#include "grasp_move.h"
GraspMove::GraspMove()
{ 
    i_arm = new MoveGroupInterface("ur5_arm");
    i_gripper = new MoveGroupInterface("gripper");
    i_scene = new PlanningSceneInterface();
}

GraspMove::~GraspMove()
{
    delete i_arm;
    delete i_gripper;
    delete i_scene;
}
void GraspMove::GoHome(MoveGroupInterface::Plan &my_plan)
{
    i_arm->setJointValueTarget(i_arm->getNamedTargetValues("home"));
    bool success = (i_arm->plan(my_plan) == MoveItErrorCode::SUCCESS);
    ROS_INFO("Step 1 %s", success ? "" : "FAILED");
    i_arm->move();
}

void GraspMove::PickPlaceOne(vector<float> &size, vector<float> &position)
{
    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

    // Add the object to be grasped (the suqare box) to the planning scene
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = i_arm->getPlanningFrame();

    if (size.size() != 3 || position.size() != 2)
    {
        return;
    }

    collision_object.id = "blue_box2";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = size[0];
    primitive.dimensions[1] = size[1];
    primitive.dimensions[2] = size[2];

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 2.0;
    box_pose.position.x = position[0];
    box_pose.position.y = position[1];
    box_pose.position.z = 1.045 - 1.21;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    i_scene->applyCollisionObjects(collision_objects);

    ROS_INFO("Add an object into the world");

    

    // Allow collisions between the gripper and the box to be able to grasp it
    planning_scene_monitor::LockedPlanningSceneRW ls(planning_scene_monitor);
    collision_detection::AllowedCollisionMatrix &acm = ls->getAllowedCollisionMatrixNonConst();
    acm.setEntry("blue_box2", "robotiq_85_left_finger_tip_link", true);
    acm.setEntry("blue_box2", "robotiq_85_right_finger_tip_link", true);
    // std::cout << "\nAllowedCollisionMatrix:\n";
    // acm.print(std::cout);

    moveit_msgs::PlanningScene diff_scene;
    ls->getPlanningSceneDiffMsg(diff_scene);

    i_scene->applyPlanningScene(diff_scene);

    // ros::Duration(0.1).sleep();

    // We can get a list of all the groups in the robot:
    ROS_INFO("Available Planning Groups:");
    std::copy(i_arm->getJointModelGroupNames().begin(),
              i_arm->getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    MoveGroupInterface::Plan my_plan;

    // 1. Move to home position
    GoHome(my_plan);

    // 2. Place the TCP (Tool Center Point, the tip of the robot) above the blue box
    geometry_msgs::PoseStamped current_pose;
    current_pose = i_arm->getCurrentPose("ee_link");
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation = current_pose.pose.orientation;
    target_pose1.position.x = position[0];
    target_pose1.position.y = position[1];
    target_pose1.position.z = 0.2;
    i_arm->setPoseTarget(target_pose1);
    bool success = (i_arm->plan(my_plan) == MoveItErrorCode::SUCCESS);
    ROS_INFO("Step 2 %s", success ? "" : "FAILED");
    i_arm->move();

    MoveGroupInterface::Plan my_plan_gripper;

    // 3. Open the gripper
    i_gripper->setJointValueTarget(i_gripper->getNamedTargetValues("open"));
    success = (i_gripper->plan(my_plan_gripper) == MoveItErrorCode::SUCCESS);
    ROS_INFO("Step 3 %s", success ? "" : "FAILED");
    i_gripper->move();

    // 4. Move the TCP close to the object
    target_pose1.position.z = target_pose1.position.z - 0.2;
    i_arm->setPoseTarget(target_pose1);
    success = (i_arm->plan(my_plan) == MoveItErrorCode::SUCCESS);
    ROS_INFO("Step 4 %s", success ? "" : "FAILED");
    i_arm->move();

    // 5. Close the  gripper
    i_gripper->setJointValueTarget(i_gripper->getNamedTargetValues("closed"));
    success = (i_gripper->plan(my_plan_gripper) == MoveItErrorCode::SUCCESS);
    ROS_INFO("Step 5 %s", success ? "" : "FAILED");
    i_gripper->move();

    // Attach the box to the gripper after it was grasped
    moveit_msgs::AttachedCollisionObject aco;
    aco.object.id = collision_object.id;
    aco.link_name = "robotiq_85_right_finger_tip_link";
    aco.touch_links.push_back("robotiq_85_left_finger_tip_link");
    aco.object.operation = moveit_msgs::CollisionObject::ADD;
    i_scene->applyAttachedCollisionObject(aco);

    // 6. Move the TCP above the plate
    target_pose1.position.z = target_pose1.position.z + 0.4;
    target_pose1.position.x = target_pose1.position.x - 0.6;
    i_arm->setPoseTarget(target_pose1);
    success = (i_arm->plan(my_plan) == MoveItErrorCode::SUCCESS);
    ROS_INFO("Step 6 %s", success ? "" : "FAILED");
    i_arm->move();

    // 7. Lower the TCP above the plate
    target_pose1.position.z = target_pose1.position.z - 0.14;
    i_arm->setPoseTarget(target_pose1);
    success = (i_arm->plan(my_plan) == MoveItErrorCode::SUCCESS);
    ROS_INFO("Step 7 %s", success ? "" : "FAILED");
    i_arm->move();

    // 8. Open the gripper
    i_gripper->setJointValueTarget(i_gripper->getNamedTargetValues("open"));
    success = (i_gripper->plan(my_plan_gripper) == MoveItErrorCode::SUCCESS);
    ROS_INFO("Step 8 %s", success ? "" : "FAILED");
    i_gripper->move();

 
    ROS_INFO("Remove the object from the world");
    std::vector<std::string> object_ids;
    object_ids.push_back(collision_object.id);
    i_scene->removeCollisionObjects(object_ids);
}

void GraspMove::PickPlace()
{
    vector<float> size = {0.06, 0.06, 0.06};
    vector<float> position = {0.3, 0.4};
    PickPlaceOne(size, position);

    size = {0.042496, 0.057710, 0.059872};
    position = {0.3, 0.6};
    PickPlaceOne(size, position);
}