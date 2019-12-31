#include <ros/ros.h>
#include <std_msgs/String.h>
#include <grasp_execution/SimpleAutomatedGraspFromTop.h>
#include <grasp_execution/SimpleAutomatedGraspFromFile.h>
#include <grasp_execution/SimpleAutomatedGraspOnlinePlanning.h>


int executeGrasp(const std::string& objectToGrasp) 
{
    ROS_INFO("Attempting grasp on: [%s]", objectToGrasp.c_str());
    // Setup grasp
    grasp_execution::SimpleAutomatedGraspExecution * graspExe;
    graspExe = new grasp_execution::SimpleAutomatedGraspFromTop();

    // Make sure we can execute grasp
    if (!graspExe)
      {
        ROS_ERROR("Unknown run type");
        return 1;
      }

    //ROS_INFO("The following object was selected: [%s]", msg->data.c_str());

    // Attempt grasp
    if (!graspExe->init() || !graspExe->graspHomeAndUngrasp("cube1"))
      {
        ROS_ERROR("Failed to run automated grasp execution");
      }
    
    if (!graspExe->init() || !graspExe->graspHomeAndUngrasp("cube2"))
      {
        ROS_ERROR("Failed to run automated grasp execution");
      }
    

    return 0;
}

void selectCube(const std_msgs::String::ConstPtr& msg)
{ 
    ROS_INFO("Object Selected: [%s]", msg->data.c_str());
    // Depending on which object is clicked in Unity
    std::string objectToGrasp;
    if (msg->data == "cube1") 
    {
      // Red
      objectToGrasp = "cube1";
    }

    else if (msg->data == "cube2") 
    {
      // Blue
      objectToGrasp = "cube2";
    }

    // Set up and execute grasp
    executeGrasp(objectToGrasp);
}

  int main(int argc, char **argv)
{
    // Create node to get info from the cube interface in Unity
    //ros::init(argc, argv, "cube_selection_node");
    ros::init(argc, argv, "grasp_action_client");

    // Subscribe to topic
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("cube_selection", 1000, selectCube);

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
    
    std::string GRASP_FILENAME;
    priv.getParam("grasp_filename",GRASP_FILENAME);

    ros::spin();
    return 0;
}