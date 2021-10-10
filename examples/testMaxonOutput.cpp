#include <iostream>
#include <maxonControl/maxonInterface.h>

int main() {

  
  std::string device_name = "snake";
  std::string config_file = "/home/paul/Desktop/util/maxonControl/maxonSetupOne.json";

  //set up interface
  maxonInterface controller(device_name);
  controller.initialize();
  controller.addNodes(config_file);

  // testing connection
  std::cout << "Hit enter and Maxon will move 2mm" << std::endl;
  std::cin.get();
  std::vector<double> pos;
  pos.push_back(2.0);
  controller.moveToPosition(pos, false);

  // configure outputs

  // call just configure
  std::vector<int> output;
  output.push_back(1);
  controller.setDigitalOutput(output);

}

