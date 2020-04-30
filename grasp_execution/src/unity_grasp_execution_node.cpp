#include <ros/ros.h>
#include <std_msgs/String.h>
#include <grasp_execution/SimpleAutomatedGraspFromTop.h>
#include <grasp_execution/SimpleAutomatedGraspFromFile.h>
#include <grasp_execution/SimpleAutomatedGraspOnlinePlanning.h>

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
    //priv.getParam("object_name2",OBJECT_NAME2);

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

    ros::spin();
    //executeGrasp("Box1"); 

    return 0;
}