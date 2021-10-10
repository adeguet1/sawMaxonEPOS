/*!
 * This is a test file for the Maxon motor.
 * 
 * This will start the motor, move at various velocities for a short time, and stop.
 */

#include "maxonInterface.h"
#include "maxonMotor.h"
#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <iostream>

#ifndef WIN32
#include <unistd.h>
#endif

int main(int argc, char **argv)
{

  if(argc < 2) {
    std::cerr << "Usage: maxonTest config_file" << std::endl;
    return 0;
  }

  maxonInterface mi("maxonInterface");
  maxonMotor *m;

  if(!mi.initialize()) {
    std::cout << "Could not initialize the motor. Quitting..." << std::endl;
    std::cout << mi.getErrorString() << std::endl;
    return 0;
  }

  if(!mi.addNodes(std::string(argv[1]))) {
    std::cout << "Could not add nodes. Quitting..." << std::endl;
    return 0;
  }

  // start the components
  mtsManagerLocal *componentManager = mtsManagerLocal::GetInstance();
  mi.Create();
  //componentManager->CreateAll();
  componentManager->WaitForStateAll(mtsComponentState::READY, 2.0*cmn_s);
  mi.Start();

  vctDoubleVec pos;
  pos.SetSize(2);
  pos.SetAll(1.0);

  mtsBool t = true;
  mi.getMotor(0)->enable(t);
  mi.getMotor(1)->enable(t);
  mi.setVelocity_raw(500);
  mi.moveToPosition(pos);

  osaSleep(2.0*cmn_s);

  //mi.moveToPosition(-pos);


  for(int i=10; i<mi.getNumMotors(); i++) {
    m = mi.getMotor(i);

    mtsBool success;
    m->enable(success);
    if(!success) {
      std::cout << "Could not enable node " << i << ". Quitting..." << std::endl;
      mtsStdString s;
      m->getErrorString(s);
      std::cout << s.Data << std::endl;
      return 0;
    }

    m->setVelocityMode();
    m->setVelocity_raw(2000);
    m->jogPlus(success);
    if(!success) {
      std::cout << "Could not move with velocity. Quitting..." << std::endl;
      mtsStdString s;
      m->getErrorString(s);
      std::cout << s.Data << std::endl;
      //return 0;
    }

  #ifdef WIN32
    Sleep(3000);
  #else
    sleep(3);
  #endif
    m->stop();
  }

  componentManager->KillAll();
  componentManager->WaitForStateAll(mtsComponentState::FINISHED, 2.0*cmn_s);
  componentManager->Cleanup();
}
