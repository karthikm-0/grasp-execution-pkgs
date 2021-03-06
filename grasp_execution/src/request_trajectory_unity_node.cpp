#include <ros/ros.h>
#include <std_msgs/String.h>
#include <grasp_execution/SimpleAutomatedGraspFromTop.h>
#include <grasp_execution/SimpleAutomatedGraspFromFile.h>
#include <grasp_execution/SimpleAutomatedGraspOnlinePlanning.h>
#include <grasp_execution/RequestTrajectory.h>
#include <grasp_execution/RequestPlacementTrajectory.h>

std::string GRASP_FILENAME;

enum AllocatedMode { Individual, Selection };
AllocatedMode mode = Selection;
std::list<std::string> allocatedObjects;

int executeGrasp(const std::string& objectToGrasp) 
{
    ROS_INFO("Attempting grasp on: [%s]", objectToGrasp.c_str());
    // Setup grasp
    grasp_execution::SimpleAutomatedGraspExecution * graspExe;
    graspExe = new grasp_execution::SimpleAutomatedGraspFromTop();
    //graspExe = new grasp_execution::SimpleAutomatedGraspFromFile(GRASP_FILENAME);

    // Make sure we can execute grasp
    if (!graspExe)
      {
        ROS_ERROR("Unknown run type");
        return 1;
      }

    //ROS_INFO("The following object was selected: [%s]", msg->data.c_str());
    
    if (!graspExe->init() || !graspExe->graspHomeAndUngrasp(objectToGrasp))
      {
        ROS_ERROR("Failed to run automated grasp execution");
      }
    // Attempt grasp
    /*if (!graspExe->init() || !graspExe->graspHomeAndUngrasp("cube1"))
      {
        ROS_ERROR("Failed to run automated grasp execution");
      }
    
    if (!graspExe->init() || !graspExe->graspHomeAndUngrasp("cube2"))
      {
        ROS_ERROR("Failed to run automated grasp execution");
      }*/
    
    return 0;
}

int homing()
{
    ROS_INFO("Attempting to home arm");
    grasp_execution::SimpleAutomatedGraspExecution * graspExe;
    graspExe = new grasp_execution::SimpleAutomatedGraspFromTop();
    if (!graspExe)
      {
        ROS_ERROR("Unknown run type");
        return 1;
      }

    if (!graspExe->init() || !graspExe->homeArm())
      {
        ROS_ERROR("Failed to home arm");
      }
}

void selectCube(const std_msgs::String::ConstPtr& msg)
{ 
    ROS_INFO("Object Selected: [%s]", msg->data.c_str());
    // Depending on which object is clicked in Unity
    std::string objectToGrasp;

    if(msg->data == "h")
    {
      homing();
    }

    else 
    {
      switch(mode)
      {
        case AllocatedMode::Individual:
          objectToGrasp = msg->data;
          executeGrasp(objectToGrasp);
          break;

        case AllocatedMode::Selection:
          // Check if the user has pressed a button that is specifically 
          if (msg->data == "e")
          {
            ROS_INFO_STREAM("Executing.");
            while (!allocatedObjects.empty())
            { 
              auto obj = allocatedObjects.front();
              allocatedObjects.pop_front();
              executeGrasp(obj);
              /*if(executeGrasp(allocatedObjects.front()))
              {
                allocatedObjects.pop_front();
              }*/
            }
            /*for (int i = 0; i < allocatedObjects.size(); i++)
            {
              if(executeGrasp(allocatedObjects.front()))
              {
                allocatedObjects.pop_front();
              }
            }*/
          }
          
          else 
          {
            ROS_INFO_STREAM("Added object to list.");
            ROS_INFO_STREAM(msg->data);
            allocatedObjects.push_back(msg->data);
          }
          
          break;
      }

    }

    /*if (msg->data == "cube1") 
    {
      // Red
      objectToGrasp = "cube1";
    }

    else if (msg->data == "cube2") 
    {
      // Blue
      objectToGrasp = "cube2";
    }*/

    // Set up and execute grasp
}

bool placement_trajectory(grasp_execution::RequestPlacementTrajectory::Request &req,
grasp_execution::RequestPlacementTrajectory::Response &res)
{
  grasp_execution::SimpleAutomatedGraspExecution * graspExe;
  graspExe = new grasp_execution::SimpleAutomatedGraspFromTop();

  // Make sure we can execute grasp
  if (!graspExe)
  {
      ROS_ERROR("Unknown run type");
      return false;
  }

  // Plan grasp and reach
  if(!graspExe->init())
  {
    ROS_ERROR("Failed to run automated grasp execution");
    return false;
  }

  else
  {
    res.robotTrajectory = graspExe->planPlacement(req.pose);
  }

  ROS_INFO_STREAM(res.robotTrajectory.joint_trajectory);
  ROS_INFO("Success, you have requested placement trajectory!");

  // Execute the reach plan
  std::string obj_name = "Random";
  graspExe->executeReach(res.robotTrajectory, obj_name);

  return true;
}

bool trajectory(grasp_execution::RequestTrajectory::Request &req,
grasp_execution::RequestTrajectory::Response &res)
{
  grasp_execution::SimpleAutomatedGraspExecution * graspExe;
  graspExe = new grasp_execution::SimpleAutomatedGraspFromTop();
  
  // Make sure we can execute grasp
  if (!graspExe)
  {
      ROS_ERROR("Unknown run type");
      return false;
  }

  /*if (!graspExe->init() || !graspExe->graspHomeAndUngrasp(req.name))
  {
      ROS_ERROR("Failed to run automated grasp execution");
      return false;
  }*/

  // Plan grasp and reach
  if(!graspExe->init())
  {
    ROS_ERROR("Failed to run automated grasp execution");
    return false;
  }

  else
  {
    res.robotTrajectory = graspExe->graspAndReachPlan(req.name);
  }

  ROS_INFO_STREAM(res.robotTrajectory.joint_trajectory);
  ROS_INFO("Success, you have requested reach trajectory!");

  // Execute the reach plan
  graspExe->executeReach(res.robotTrajectory, req.name);

  return true;
}

int main(int argc, char **argv)
{
    // Create node to get info from the cube interface in Unity
    //ros::init(argc, argv, "cube_selection_node");
    ros::init(argc, argv, "grasp_action_client");

    // Subscribe to topic
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("cube_selection", 1, selectCube);

    // Create server for grasping
    ros::NodeHandle priv("~");
    std::string OBJECT_NAME;
    std::string OBJECT_NAME2;

    if (!priv.hasParam("object_name"))
    {
      ROS_ERROR("have to specify 'object_name' as ROS parameter");
      return 0;
    }
    priv.getParam("object_name",OBJECT_NAME);

    int RUN_TYPE;
    if (!priv.hasParam("run_type"))
    {
        ROS_ERROR("have to specify 'run_type' as ROS parameter");
        return 0;
    }
    priv.getParam("run_type",RUN_TYPE);
    
    //std::string GRASP_FILENAME;
    priv.getParam("grasp_filename",GRASP_FILENAME);
    if ((RUN_TYPE==2) && (!priv.hasParam("grasp_filename") || GRASP_FILENAME.empty()))
    {
      {
        ROS_ERROR("run_type = 2 requires additional specification of 'grasp_filename'");
        return 0;
      }
    }

    // Launch a service to request trajectories for reaching and then for placing
    ros::ServiceServer service = n.advertiseService("request_trajectories_srv", trajectory);
    
    ros::ServiceServer service_two = n.advertiseService("request_placement_trajectories_srv", placement_trajectory);


    //executeGrasp("Box1"); 
    ros::spin();
    
    return 0;
}