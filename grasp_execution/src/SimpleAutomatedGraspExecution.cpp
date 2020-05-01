#include <grasp_execution/SimpleAutomatedGraspExecution.h>
#include <convenience_ros_functions/ROSFunctions.h>
#include <convenience_math_functions/MathFunctions.h>

#include <object_msgs/Object.h>
#include <object_msgs/ObjectInfo.h>
#include <object_msgs_tools/ObjectFunctions.h>
#include <ros/ros.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <gazebo_ros_link_attacher/AttachRequest.h>
#include <gazebo_ros_link_attacher/AttachResponse.h>

#include <gazebo_msgs/SetPhysicsProperties.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SetLinkProperties.h>

#include <gazebo_msgs/GetModelState.h>

#include <moveit_msgs/RobotTrajectory.h>
#include <tf/transform_datatypes.h>

////

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

using grasp_execution::SimpleAutomatedGraspExecution;

using object_msgs_tools::ObjectFunctions;
using moveit_planning_helper::MoveItPlanner;
using convenience_ros_functions::RobotInfo;
using convenience_math_functions::MathFunctions;
using arm_components_name_manager::ArmComponentsNameManager;


SimpleAutomatedGraspExecution::SimpleAutomatedGraspExecution():
    initialized(false),
    jointsManager(NULL),
    trajectoryPlanner(NULL),
    graspActionClient(NULL),
    jointTrajectoryActionClient(NULL),
    graspHandler(NULL)
{
    convenience_ros_functions::ROSFunctions::initSingleton();
}
SimpleAutomatedGraspExecution::~SimpleAutomatedGraspExecution()
{
    if (jointsManager) delete jointsManager;
    if (trajectoryPlanner) delete trajectoryPlanner;
    if (graspActionClient) delete graspActionClient;
    if (jointTrajectoryActionClient) delete jointTrajectoryActionClient;
    if (graspHandler) delete graspHandler;    
}

bool SimpleAutomatedGraspExecution::init()
{
    ros::NodeHandle priv("~");

    /////////// Read parameters  ///////////////////
    std::string ROBOT_NAMESPACE;
    if (!priv.hasParam("robot_namespace"))
    {
        ROS_ERROR("Node requires private parameter 'robot_namespace'");
        return false;
    }
    priv.param<std::string>("robot_namespace", ROBOT_NAMESPACE, ROBOT_NAMESPACE);

    jointsManager = new ArmComponentsNameManager(ROBOT_NAMESPACE, false);
    double maxWait=5;
    ROS_INFO("Waiting for joint info parameters to be loaded...");
    jointsManager->waitToLoadParameters(1,maxWait); 
    ROS_INFO("Parameters loaded.");

    OPEN_ANGLES=0.05;
    priv.param<double>("open_angles", OPEN_ANGLES, OPEN_ANGLES);
    CLOSE_ANGLES=0.7;
    priv.param<double>("close_angles", CLOSE_ANGLES, CLOSE_ANGLES);
  
    EFF_POS_TOL=0.02;
    EFF_ORI_TOL=0.1;
    JOINT_ANGLE_TOL=0.1;
    PLAN_TOLERANCE_FACTOR=0.1;
    priv.param<double>("effector_pos_tolerance", EFF_POS_TOL, EFF_POS_TOL);
    priv.param<double>("effector_ori_tolerance", EFF_ORI_TOL, EFF_ORI_TOL);
    priv.param<double>("joint_angle_tolerance", JOINT_ANGLE_TOL, JOINT_ANGLE_TOL);
    priv.param<double>("plan_tolerance_factor", PLAN_TOLERANCE_FACTOR, PLAN_TOLERANCE_FACTOR);

    REQUEST_OBJECTS_SERVICE="world/request_object";
    priv.param<std::string>("request_object_service", REQUEST_OBJECTS_SERVICE, REQUEST_OBJECTS_SERVICE);
    
    JOINT_STATES_TOPIC = "/joint_states";
    priv.param<std::string>("joint_states_topic", JOINT_STATES_TOPIC, JOINT_STATES_TOPIC);

    GRASP_ACTION_NAME = "/grasp_action";
    priv.param<std::string>("grasp_action_name", GRASP_ACTION_NAME, GRASP_ACTION_NAME);
    
    JOINT_TRAJECTORY_ACTION_NAME = "/joint_trajectory_action";
    priv.param<std::string>("joint_trajectory_action_name", JOINT_TRAJECTORY_ACTION_NAME, JOINT_TRAJECTORY_ACTION_NAME);

    MOVEIT_MOTION_PLAN_SERVICE="/plan_kinematic_path";
    priv.param<std::string>("moveit_motion_plan_service", MOVEIT_MOTION_PLAN_SERVICE, MOVEIT_MOTION_PLAN_SERVICE);

    MOVEIT_STATE_VALIDITY_SERVICE="/check_state_validity";
    priv.param<std::string>("moveit_state_validity_service", MOVEIT_STATE_VALIDITY_SERVICE, MOVEIT_STATE_VALIDITY_SERVICE);

    MOVEIT_GET_PLANNING_SCENE_SERVICE="/get_planning_scene";
    priv.param<std::string>("moveit_get_planning_scene_service", MOVEIT_GET_PLANNING_SCENE_SERVICE, MOVEIT_GET_PLANNING_SCENE_SERVICE);
    
    MOVEIT_SET_PLANNING_SCENE_TOPIC="/planning_scene";
    priv.param<std::string>("moveit_get_planning_scene_topic", MOVEIT_SET_PLANNING_SCENE_TOPIC, MOVEIT_SET_PLANNING_SCENE_TOPIC);

    ARM_REACH_SPAN = 2;
    priv.param<double>("arm_reach_span", ARM_REACH_SPAN, ARM_REACH_SPAN);
    PLANNING_GROUP="Arm";
    priv.param<std::string>("planning_group", PLANNING_GROUP, PLANNING_GROUP);

    trajectoryPlanner = new MoveItPlanner(node, 
        MOVEIT_MOTION_PLAN_SERVICE,
        MOVEIT_STATE_VALIDITY_SERVICE);

    //create the action client
    // true causes the client to spin its own thread
    graspActionClient = new actionlib::SimpleActionClient<grasp_execution_msgs::GraspAction> (GRASP_ACTION_NAME, true);
    ROS_INFO_STREAM("Waiting for grasp action client to start: "<< GRASP_ACTION_NAME);
    // wait for the action server to start
    graspActionClient->waitForServer(); //will wait for infinite time
    ROS_INFO("Grasp action client started.");

    // create the action client
    jointTrajectoryActionClient = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(JOINT_TRAJECTORY_ACTION_NAME, true);
    ROS_INFO_STREAM("Waiting for joint trajectory action client to start: "<< JOINT_TRAJECTORY_ACTION_NAME);
    // wait for the action server to start
    jointTrajectoryActionClient->waitForServer();  // will wait for infinite time
    ROS_INFO("Joint trajectory action client has started.");

    std::vector<std::string> gripperLinkNames=jointsManager->getGripperLinks();
    std::string palmLinkName = jointsManager->getPalmLink();
    gripperLinkNames.push_back(palmLinkName);
    
    graspHandler = new moveit_object_handling::GraspedObjectHandlerMoveIt(node,
        gripperLinkNames, MOVEIT_GET_PLANNING_SCENE_SERVICE,MOVEIT_SET_PLANNING_SCENE_TOPIC);

    graspHandler->waitForSubscribers();

    initialized = initImpl();
    return initialized;
}

bool SimpleAutomatedGraspExecution::getObjectPose(const std::string& object_name, const float timeout_wait_object, geometry_msgs::PoseStamped& pose)
{
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<object_msgs::ObjectInfo>(REQUEST_OBJECTS_SERVICE);
    object_msgs::ObjectInfo srv;
    srv.request.name = object_name;
    srv.request.get_geometry=false;

    ros::Time startTime = ros::Time::now();
    float timeWaited = 0;
    bool success=false;
    while (timeWaited < timeout_wait_object)
    {
        if (client.call(srv) && srv.response.success)
        {
            // ROS_INFO("getObjectInfo result:");
            // std::cout<<srv.response<<std::endl;
            success = true;
            break;
        }
        ros::Time currTime = ros::Time::now();
        timeWaited = (currTime - startTime).toSec();
    }
    if (!success)
    {
        ROS_ERROR("Failed to call service %s, error code: %i",REQUEST_OBJECTS_SERVICE.c_str(),srv.response.error_code);
        return false;
    }
    return ObjectFunctions::getObjectPose(srv.response.object,pose);
}


sensor_msgs::JointState SimpleAutomatedGraspExecution::getHomeState(bool capToPI)
{
    std::vector<float> arm_init = jointsManager->getArmJointsInitPose();
    if (capToPI)
    {
        MathFunctions::capToPI(arm_init); 
    }
    sensor_msgs::JointState ret;
    jointsManager->copyToJointState(ret, 0, &arm_init, 0, true);
    ret.velocity.resize(arm_init.size(),0);
    return ret;
}

bool SimpleAutomatedGraspExecution::setFingersToCurr(const sensor_msgs::JointState& currState, sensor_msgs::JointState& targetState)
{
    std::vector<std::string> gripperJoints = jointsManager->getGripperJoints();
    std::vector<int> idxCurr;
    if (!jointsManager->getJointIndices(currState.name, idxCurr,2))
    {
        ROS_ERROR("Current state does not have finger joint names.");
        return false;
    }
        
    if (idxCurr.size() != gripperJoints.size())
    {
        ROS_ERROR_STREAM("Consistency in current state: number of joint indices ("<<idxCurr.size()<<
            ") smaller than expected ("<<gripperJoints.size()<<")");
        return false;
    }

    std::vector<int> idxTarget;
    if (!jointsManager->getJointIndices(targetState.name, idxTarget,2))
    {
        ROS_ERROR("Current state does not have finger joint names.");
        return false;
    }
        
    if (idxTarget.size() != gripperJoints.size())
    {
        ROS_ERROR_STREAM("Consistency in target state: number of joint indices ("<<idxTarget.size()<<
            ") smaller than expected ("<<gripperJoints.size()<<")");
        return false;
    }


    for (int i=0; i < gripperJoints.size(); ++i)
    {
        targetState.position[idxTarget[i]]=currState.position[idxCurr[i]];
    }
    return true;
}

int SimpleAutomatedGraspExecution::planAndExecuteMotion(
    const std::string& fixed_frame_id,
    const std::string& arm_base_link,
    moveit_msgs::Constraints& reachConstraints,
    const float arm_reach_span,
    const std::string& planning_group)
{
    // get the current arm base pose.
    geometry_msgs::PoseStamped currBasePose;
    int transRet=convenience_ros_functions::ROSFunctions::Singleton()->getTransform(
            fixed_frame_id, arm_base_link,
            currBasePose.pose,
            ros::Time(0),2,true);
    if (transRet!=0) {
        ROS_ERROR("Could not get current effector tf transform in object frame.");
        return -1;
    }
    currBasePose.header.stamp=ros::Time::now();
    currBasePose.header.frame_id=fixed_frame_id;
    // ROS_INFO_STREAM("Effector currBasePose pose: "<<currBasePose);
 
    // request joint trajectory
    sensor_msgs::JointState currArmJointState = robotInfo.getCurrentJointState(JOINT_STATES_TOPIC, node);
    // get only the arm joints of the joint state:
    jointsManager->extractFromJointState(currArmJointState,0,currArmJointState);
    // ROS_INFO_STREAM("Current arm joint state: "<<currArmJointState);

    ROS_INFO("Planning trajectory...");
    moveit_msgs::RobotTrajectory robotTrajectory;
    moveit_msgs::MoveItErrorCodes moveitRet = trajectoryPlanner->requestTrajectory(
        currBasePose,
        arm_reach_span,
        planning_group,
        reachConstraints,
        NULL,
        currArmJointState,
        robotTrajectory);

    if (moveitRet.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
        ROS_ERROR("Could not plan joint trajectory");
        return -2;
    }

    else
    {
        //ROS_INFO_STREAM(robotTrajectory.joint_trajectory);
    }


    // ROS_INFO("############  Resulting joint trajectory ################");
    // ROS_INFO_STREAM(robotTrajectory.joint_trajectory);

    /////////// Execute joint trajectory  ///////////////////

    //ROS_INFO("Now constructing joint trajectory goal");

    // send a goal to the action
    control_msgs::FollowJointTrajectoryGoal jtGoal;
    jtGoal.trajectory = robotTrajectory.joint_trajectory;

    ROS_INFO("Now sending joint trajectory goal");
    jointTrajectoryActionClient->sendGoal(jtGoal);

    // wait for the action to return
    bool finished_before_timeout = jointTrajectoryActionClient->waitForResult(ros::Duration(15.0));
    actionlib::SimpleClientGoalState state = jointTrajectoryActionClient->getState();
    if (state!=actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        if (finished_before_timeout)
        {
            ROS_ERROR("Could not execute joint trajectory action: %s",state.toString().c_str());
        }
        else
        {
            ROS_ERROR("Joint trajectory action did not finish before the time out.");
        }
        return -3;
    }

    ROS_INFO("Joint trajectory action finished: %s",state.toString().c_str());
    return 0;
}

moveit_msgs::RobotTrajectory SimpleAutomatedGraspExecution::planMotion(
    const std::string& fixed_frame_id,
    const std::string& arm_base_link,
    moveit_msgs::Constraints& reachConstraints,
    const float arm_reach_span,
    const std::string& planning_group)
{
    // Final trajectory we will send out
    moveit_msgs::RobotTrajectory robotTrajectory;

    // get the current arm base pose.
    geometry_msgs::PoseStamped currBasePose;
    int transRet=convenience_ros_functions::ROSFunctions::Singleton()->getTransform(
            fixed_frame_id, arm_base_link,
            currBasePose.pose,
            ros::Time(0),2,true);
    if (transRet!=0) {
        ROS_ERROR("Could not get current effector tf transform in object frame.");
        return robotTrajectory;
    }
    
    currBasePose.header.stamp=ros::Time::now();
    currBasePose.header.frame_id=fixed_frame_id;
    // ROS_INFO_STREAM("Effector currBasePose pose: "<<currBasePose);
 
    // request joint trajectory
    sensor_msgs::JointState currArmJointState = robotInfo.getCurrentJointState(JOINT_STATES_TOPIC, node);
    // get only the arm joints of the joint state:
    jointsManager->extractFromJointState(currArmJointState,0,currArmJointState);
    // ROS_INFO_STREAM("Current arm joint state: "<<currArmJointState);

    ROS_INFO("Planning trajectory...");
    moveit_msgs::MoveItErrorCodes moveitRet = trajectoryPlanner->requestTrajectory(
        currBasePose,
        arm_reach_span,
        planning_group,
        reachConstraints,
        NULL,
        currArmJointState,
        robotTrajectory);

    if (moveitRet.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
        ROS_ERROR("Could not plan joint trajectory");
        return robotTrajectory;
    }

    else
    {
        ROS_INFO_STREAM(robotTrajectory.joint_trajectory);
    }

    return robotTrajectory;
}

bool SimpleAutomatedGraspExecution::graspPlan(const std::string& object_name, bool doGrasp, grasp_execution_msgs::GraspGoal& graspGoal)
{
    return getGrasp(object_name, doGrasp, graspGoal);
}

bool SimpleAutomatedGraspExecution::reach(const std::string& object_name, const grasp_execution_msgs::GraspGoal& graspGoal)
{
    std::string effector_link = jointsManager->getEffectorLink();
    std::string arm_base_link = jointsManager->getArmLinks().front();
    std::string object_frame_id = object_name;
  
    // build planning constraints:
    float plan_eff_pos_tol = EFF_POS_TOL * PLAN_TOLERANCE_FACTOR;
    float plan_eff_ori_tol = EFF_ORI_TOL * PLAN_TOLERANCE_FACTOR;
    int type = 1; // 0 = only position, 1 = pos and ori, 2 = only ori
    moveit_msgs::Constraints reachConstraints = trajectoryPlanner->getPoseConstraint(effector_link,
        graspGoal.grasp.grasp.grasp_pose, plan_eff_pos_tol, plan_eff_ori_tol, type); 

    int motionRet = planAndExecuteMotion(
        object_frame_id,
        arm_base_link,
        reachConstraints,
        ARM_REACH_SPAN,
        PLANNING_GROUP);
    if (motionRet !=0)
    {
        ROS_ERROR_STREAM("Could not plan/execution motion, return code "<<motionRet);
        return false;
    }
    return true;
}

moveit_msgs::RobotTrajectory SimpleAutomatedGraspExecution::reachPlan(const std::string& object_name, const grasp_execution_msgs::GraspGoal& graspGoal)
{
    std::string effector_link = jointsManager->getEffectorLink();
    std::string arm_base_link = jointsManager->getArmLinks().front();
    std::string object_frame_id = object_name;
  
    // build planning constraints:
    float plan_eff_pos_tol = EFF_POS_TOL * PLAN_TOLERANCE_FACTOR;
    float plan_eff_ori_tol = EFF_ORI_TOL * PLAN_TOLERANCE_FACTOR;
    int type = 1; // 0 = only position, 1 = pos and ori, 2 = only ori
    moveit_msgs::Constraints reachConstraints = trajectoryPlanner->getPoseConstraint(effector_link,
        graspGoal.grasp.grasp.grasp_pose, plan_eff_pos_tol, plan_eff_ori_tol, type); 

    moveit_msgs::RobotTrajectory trajectory = planMotion(
        object_frame_id,
        arm_base_link,
        reachConstraints,
        ARM_REACH_SPAN,
        PLANNING_GROUP);

    return trajectory;
}

bool SimpleAutomatedGraspExecution::executeReach(moveit_msgs::RobotTrajectory trajectory, const std::string& object_name)
{
     // send a goal to the action
    control_msgs::FollowJointTrajectoryGoal jtGoal;
    jtGoal.trajectory = trajectory.joint_trajectory;

    ROS_INFO("Now sending joint trajectory goal");
    jointTrajectoryActionClient->sendGoal(jtGoal);

    // wait for the action to return
    bool finished_before_timeout = jointTrajectoryActionClient->waitForResult(ros::Duration(15.0));
    actionlib::SimpleClientGoalState state = jointTrajectoryActionClient->getState();
    if (state!=actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        if (finished_before_timeout)
        {
            ROS_ERROR("Could not execute joint trajectory action: %s",state.toString().c_str());
        }
        else
        {
            ROS_ERROR("Joint trajectory action did not finish before the time out.");
        }
        return -3;
    }

    ROS_INFO("Joint trajectory action finished: %s",state.toString().c_str());

    fakeGrasp(object_name);

    ROS_INFO("Finished grasping object");

    return 0; 
}

    
bool SimpleAutomatedGraspExecution::grasp(const std::string& object_name, const grasp_execution_msgs::GraspGoal& graspGoal)
{
    /*graspActionClient->sendGoal(graspGoal);

    //wait for the action to return
    bool finished_before_timeout = graspActionClient->waitForResult(ros::Duration(15.0));
    actionlib::SimpleClientGoalState state = graspActionClient->getState();
    if (state!=actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        if (finished_before_timeout)
        {
            ROS_ERROR("Could not execute grasp action: %s",state.toString().c_str());
        }
        else
        {
            ROS_ERROR("Grasp action did not finish before the time out.");
        }
        return false;
    }
    ROS_INFO("Grasp action finished: %s",state.toString().c_str());*/

    // Attach object to MoveIt! robot
    std::string palmLinkName = jointsManager->getPalmLink();
    graspHandler->attachObjectToRobot(object_name,palmLinkName);

    // Elevate object slightly
    // Get model's current pose first
    ros::NodeHandle n;
    ros::ServiceClient gz_client1 = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState getSrv;
    getSrv.request.model_name = object_name;
    getSrv.request.relative_entity_name = "world";
    gz_client1.call(getSrv);
    geometry_msgs::Pose obj_pose = getSrv.response.pose;

    // Set the pose next (slightly elevated)
    ros::ServiceClient gz_client2 = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState setSrv;
    setSrv.request.model_state.model_name = object_name;
    setSrv.request.model_state.pose.position.x = obj_pose.position.x;
    setSrv.request.model_state.pose.position.y = obj_pose.position.y;
    setSrv.request.model_state.pose.position.z = 0.81;  
    setSrv.request.model_state.pose.orientation.w = obj_pose.orientation.w;
    setSrv.request.model_state.pose.orientation.x = obj_pose.orientation.x;
    setSrv.request.model_state.pose.orientation.y = obj_pose.orientation.y;
    setSrv.request.model_state.pose.orientation.z = obj_pose.orientation.z;
    ROS_INFO_STREAM(gz_client2.call(setSrv));

    // Remove gravity
    //Set_model_state azebo::GazeboRosApiPlugin::setPhysicsProperties
    ros::ServiceClient grav_client = n.serviceClient<gazebo_msgs::SetLinkProperties>("/gazebo/set_link_properties");
    gazebo_msgs::SetLinkProperties removeSrv;
    std::string link_name = object_name + std::string("::link");
    removeSrv.request.link_name = link_name;
    removeSrv.request.gravity_mode = 0;
    ROS_INFO_STREAM(grav_client.call(removeSrv));

    // Attach the correct links together (of the cube and arm)
    //ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    gazebo_ros_link_attacher::Attach attachSrv;
    attachSrv.request.model_name_1 = "jaco_on_table";
    attachSrv.request.link_name_1 = "jaco_6_hand_limb";
    attachSrv.request.model_name_2 = object_name;
    attachSrv.request.link_name_2 = "link";
    ROS_INFO_STREAM(client.call(attachSrv));

    return true;

    /*if(client.call(attachSrv.request))
    {
        ROS_INFO_STREAM(attachSrv.response);
        return true;
    }
    else 
    {
        return false;
    }*/
}

bool SimpleAutomatedGraspExecution::fakeGrasp(const std::string& object_name)
{
    std::string palmLinkName = jointsManager->getPalmLink();
    graspHandler->attachObjectToRobot(object_name,palmLinkName);
    
    return true;
}

bool SimpleAutomatedGraspExecution::unGrasp(const std::string& object_name, const grasp_execution_msgs::GraspGoal& graspGoal)
{
    /*graspActionClient->sendGoal(graspGoal);

    //wait for the action to return
    bool finished_before_timeout = graspActionClient->waitForResult(ros::Duration(15.0));
    actionlib::SimpleClientGoalState state = graspActionClient->getState();
    if (state!=actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        if (finished_before_timeout)
        {
            ROS_ERROR("Could not execute grasp action: %s",state.toString().c_str());
        }
        else
        {
            ROS_ERROR("Grasp action did not finish before the time out.");
        }
        return false;
    }
    ROS_INFO("Grasp action finished: %s",state.toString().c_str());*/

    // dettach object to MoveIt! robot
    graspHandler->detachObjectFromRobot(object_name);

    // Detach the links between object and arm
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
    gazebo_ros_link_attacher::Attach attachSrv;
    attachSrv.request.model_name_1 = "jaco_on_table";
    attachSrv.request.link_name_1 = "jaco_6_hand_limb";
    attachSrv.request.model_name_2 = object_name;
    attachSrv.request.link_name_2 = "link";
    ROS_INFO_STREAM(client.call(attachSrv));

    // Enable gravity
    ros::ServiceClient gz_client = n.serviceClient<gazebo_msgs::SetLinkProperties>("/gazebo/set_link_properties");
    gazebo_msgs::SetLinkProperties setSrv;
    std::string link_name = object_name + std::string("::link");
    setSrv.request.link_name = link_name;
    setSrv.request.gravity_mode = 1;
    ROS_INFO_STREAM(gz_client.call(setSrv));

    return true;
}

bool SimpleAutomatedGraspExecution::homeArm()
{
    sensor_msgs::JointState homeState=getHomeState(true);
    sensor_msgs::JointState currJointState = robotInfo.getCurrentJointState(JOINT_STATES_TOPIC, node);
    if (!setFingersToCurr(currJointState, homeState))
    {
        ROS_ERROR("Could not set the target finger states");
        return false;
    }

    float JOINT_PLAN_TOL=JOINT_ANGLE_TOL;
    moveit_msgs::Constraints homeConstraints = trajectoryPlanner->getJointConstraint(homeState,JOINT_PLAN_TOL); 
    
    std::string arm_base_link = jointsManager->getArmLinks().front();

    ROS_INFO_STREAM("Planning for constraints: " << homeConstraints);

    int homeMotionRet = planAndExecuteMotion(
        arm_base_link,
        arm_base_link,
        homeConstraints,
        ARM_REACH_SPAN,
        PLANNING_GROUP);
    if (homeMotionRet !=0)
    {
        ROS_ERROR_STREAM("Could not plan/execution motion to HOME, return code "<<homeMotionRet);
        return false;
    }
    return true;
}

bool SimpleAutomatedGraspExecution::moveArmToCartesian(const std::string& object_name)
{   
    // Arbritary pose to place the object in
    std::string effector_link = jointsManager->getEffectorLink();
    std::string arm_base_link = jointsManager->getArmLinks().front();
    std::string palmLinkName = jointsManager->getPalmLink();

    geometry_msgs::PoseStamped object_target_pose;
    object_target_pose.header.frame_id = "tabletop_ontop";
    object_target_pose.pose.position.x = 0.2;
    object_target_pose.pose.position.y = 0.2;
    object_target_pose.pose.position.z = 0.2;
    object_target_pose.pose.orientation.w = 1;
    object_target_pose.pose.orientation.x = 0;
    object_target_pose.pose.orientation.y = 0;
    object_target_pose.pose.orientation.w = 0;

    // build planning constraints
    float plan_eff_pos_tol = EFF_POS_TOL * PLAN_TOLERANCE_FACTOR;
    float plan_eff_ori_tol = EFF_ORI_TOL * PLAN_TOLERANCE_FACTOR;
    int type = 1; // 0 = only position, 1 = pos and ori, 2 = only ori
    moveit_msgs::Constraints reachConstraints = trajectoryPlanner->getPoseConstraint(effector_link, 
        object_target_pose, plan_eff_pos_tol, plan_eff_ori_tol, type); 

    int motionRet = planAndExecuteMotion(
        object_target_pose.header.frame_id,
        arm_base_link,
        reachConstraints,
        ARM_REACH_SPAN,
        PLANNING_GROUP);
        
    if (motionRet !=0)
    {
        ROS_ERROR_STREAM("Could not plan/execution motion, return code "<<motionRet);
        return false;
    }
    return true;
}

moveit_msgs::RobotTrajectory SimpleAutomatedGraspExecution::planPlacement(geometry_msgs::Pose& pose)
{
    // Arbritary pose to place the object in
    std::string effector_link = jointsManager->getEffectorLink();
    std::string arm_base_link = jointsManager->getArmLinks().front();
    std::string palmLinkName = jointsManager->getPalmLink();

    ROS_INFO_STREAM(pose);
    tf::Quaternion q;
    q.setRPY(0, 55, 0);
    geometry_msgs::PoseStamped object_target_pose;
    object_target_pose.header.frame_id = "world";
    object_target_pose.pose.position.x = pose.position.x;
    object_target_pose.pose.position.y = pose.position.y;
    object_target_pose.pose.position.z = pose.position.z;
    object_target_pose.pose.orientation.x = q.x();
    object_target_pose.pose.orientation.y = q.y();
    object_target_pose.pose.orientation.z = q.z();
    object_target_pose.pose.orientation.w = q.w();

    //static const std::string PLANNING_GROUP = "jaco";
    //moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);



    /*tf::Quaternion q;
    q.setEulerZYX(0, 0.872665, 0);
    geometry_msgs::PoseStamped ee_pose;
    ee_pose.header.frame_id = "jaco_6_hand_limb";
    ee_pose.pose.position.x = 0;
    ee_pose.pose.position.y = 0;
    ee_pose.pose.position.z = 0;
    ee_pose.pose.orientation.x = q.x();
    ee_pose.pose.orientation.y = q.y();
    ee_pose.pose.orientation.z = q.z();
    ee_pose.pose.orientation.w = q.w();*/



    // build planning constraints
    float plan_eff_pos_tol = EFF_POS_TOL * PLAN_TOLERANCE_FACTOR;
    float plan_eff_ori_tol = EFF_ORI_TOL * PLAN_TOLERANCE_FACTOR;
    int type = 1; // 0 = only position, 1 = pos and ori, 2 = only ori
    moveit_msgs::Constraints reachConstraints = trajectoryPlanner->getPoseConstraint(effector_link, 
    object_target_pose, plan_eff_pos_tol, plan_eff_ori_tol, type); 

    moveit_msgs::RobotTrajectory trajectory = planMotion(
        //object_target_pose.header.frame_id,
        arm_base_link,
        arm_base_link,
        reachConstraints,
        ARM_REACH_SPAN,
        PLANNING_GROUP);

    return trajectory;
}

moveit_msgs::RobotTrajectory SimpleAutomatedGraspExecution::graspAndReachPlan(const std::string& object_name)
{
    grasp_execution_msgs::GraspGoal graspGoal;
    ROS_INFO_STREAM("###### Grasp Planning #######");
    if (!graspPlan(object_name, true, graspGoal))
    {
        ROS_ERROR_STREAM("Could not plan the grasp for "<<object_name);
    }

    ROS_INFO_STREAM("###### Planning ######");
    moveit_msgs::RobotTrajectory trajectory = reachPlan(object_name, graspGoal);  
    return trajectory; 

    ROS_INFO_STREAM("###### Finished grasp and reach planning #######");
}

/*moveit_msgs::RobotTrajectory SimpleAutomatedGraspExecution::placePlanWithEE(geometry_msgs::Pose& pose)
{
    grasp_execution_msgs::GraspGoal graspGoal;
    ROS_INFO_STREAM("###### Grasp Planning #######");
    if (!graspPlan("Box1", true, graspGoal))
    {
        ROS_ERROR_STREAM("Could not plan the grasp for "<<object_name);
    }

    ROS_INFO_STREAM("###### Planning ######");
    moveit_msgs::RobotTrajectory trajectory = planPlacement(pose)
    return trajectory; 

    ROS_INFO_STREAM("###### Finished grasp and reach planning #######");
}*/


bool SimpleAutomatedGraspExecution::graspHomeAndUngrasp(const std::string& object_name)
{
    grasp_execution_msgs::GraspGoal graspGoal;
    ROS_INFO_STREAM("###### Grasp Planning #######");
    if (!graspPlan(object_name, true, graspGoal))
    {
        ROS_ERROR_STREAM("Could not plan the grasp for "<<object_name);
        return false;
    }

    ROS_INFO_STREAM("###### Planning ######");
    moveit_msgs::RobotTrajectory trajectory = reachPlan(object_name, graspGoal);
    //ROS_INFO_STREAM(trajectory.joint_trajectory);
    

    /*ROS_INFO_STREAM("###### Reaching #######");
    if (!reach(object_name, graspGoal))
    {
        ROS_ERROR_STREAM("Could not reach to "<<object_name);
        return false;
    }*/
    
    /*ROS_INFO_STREAM("###### Grasping #######");
    if (!grasp(object_name, graspGoal))
    {
        ROS_ERROR_STREAM("Could not grasp "<<object_name);
        return false;
    }

    ROS_INFO_STREAM("###### Repositioning #######");
    if (!moveArmToCartesian(object_name))
    {
        ROS_ERROR_STREAM("Could not move to object goal state "<<object_name);
        return false;
    }

    ROS_INFO_STREAM("###### Un-Grasp Planning #######");
    grasp_execution_msgs::GraspGoal ungraspGoal;
    if (!graspPlan(object_name, false, ungraspGoal))
    {
        ROS_ERROR_STREAM("Could not plan the grasp for "<<object_name);
        return false;
    }
 
    ROS_INFO_STREAM("###### Un-Grasping #######");
    if (!unGrasp(object_name, ungraspGoal))
    {
        ROS_ERROR_STREAM("Could not un-grasp "<<object_name);
        return false;
    }

    ROS_INFO_STREAM("###### Homing arm #######");
    if (!homeArm())
    {
        ROS_ERROR_STREAM("Could not home the arm after grasping "<<object_name);
        return false;
    }*/

    return true;
}
