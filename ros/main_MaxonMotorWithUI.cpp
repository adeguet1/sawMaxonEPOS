  
#include <QApplication>

#include <ros/ros.h>
#include <cisst_ros_bridge/mtsROSBridge.h>
#include <sawMaxonEPOS/maxonMotor.h>
#include <sawMaxonEPOS/maxonInterface.h>
#include <cisstMultiTask/mtsManagerLocal.h>

#include <sawMaxonEPOS/maxonWidget.h>
#include <sawMaxonEPOS/maxonStatusWidget.h>
#include <sawMaxonEPOS/maxonControlWidget.h>


//TODO: make both position and velocity modes possible

int main (int argc, char *argv[]){
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);

    ros::init(argc, argv, "MaxonUI", ros::init_options::AnonymousName);
    ros::NodeHandle rosNodeHandle;
    std::string path_to_json = argv[1];
    std::string velocity_mode_string = argv[2]; // If "1" then velocity mode, else position mode. 
    bool velocity_mode = velocity_mode_string == "1"; 
    std::string show_UI_string = argv[3];
    bool show_UI = show_UI_string == "1";

    std::vector<std::string> motor_names;

    QApplication app(argc, argv);
    

    mtsComponentManager * manager = mtsManagerLocal::GetInstance();
    mtsROSBridge* rosBridge = new mtsROSBridge("bridge", 50.0 * cmn_ms, &rosNodeHandle);
    rosBridge->PerformsSpin(true); // Needed since using with QT, otherwise subscribers won't process
    manager->AddComponent(rosBridge);
  
    maxonInterface mi("maxonInterface",""); // this only initializes the mtsComponent class, 
    bool maxonsConnected = mi.initialize(); // this sets the controller handle for each of the motor tasks
    if(!maxonsConnected){
      std::cout << "Motors not connected, exiting" << std::endl;
      return 0;
    }
    CMN_LOG_INIT_DEBUG << "adding maxon nodes" << std::endl;
    mi.addNodes(path_to_json); // this creates the motor tasks & interfaces per config file
    
    maxonWidget mw(&mi);
    if(show_UI){
      manager->AddComponent(mw.statusWidget);
    }

    int numMotors = mi.getNumMotors();
    std::cout << "nummotors:" << numMotors<< std::endl;
    for (int i = 0; i < numMotors; i++)
      {
        maxonMotor *m = mi.getMotor(i);
        CMN_LOG_RUN_VERBOSE << "adding motor " << m->GetName() << " to manager" << std::endl;
        manager->AddComponent(m);
        mtsBool enabled;
        m->enable(enabled);
        if (velocity_mode){
          m->setVelocityMode();
          m->setSetPointMode(true);
        }
        motor_names.push_back(m->GetName());
        if(show_UI){
          manager->AddComponent(mw.controls[i]);
        }
      }
    

for(auto& motor_name : motor_names){
    rosBridge->AddPublisherFromCommandRead  // send velocity commands to ROS
    <double, std_msgs::Float32>
    (motor_name +"_required_interface_state","position",
    "/motor_"+motor_name+"/measured_js");

    if (velocity_mode){
      rosBridge->AddSubscriberToCommandWrite  // send velocity commands to ROS
      <double, std_msgs::Float32>
      (motor_name +"_required_interface_control","setVelocityPoint",
      "/motor_"+motor_name+"/servo_jv");
    }
    //else{
      rosBridge->AddSubscriberToCommandWrite  // send position commands to ROS
      <double, std_msgs::Float32>
      (motor_name +"_required_interface_control","moveToAbsolutePosition",
      "/motor_"+motor_name+"/move_jp");
   // }
    manager->Connect(rosBridge->GetName(),motor_name + "_required_interface_state",motor_name,"state");
    manager->Connect(rosBridge->GetName(),motor_name + "_required_interface_control",motor_name,"control");
}
    if(show_UI){
      mw.Connect(manager);
    }
    
    manager->CreateAll();
    manager->StartAll();

    if(show_UI){
      mw.show();
      app.exec();
    }
    else
      {ros::spin();}
}