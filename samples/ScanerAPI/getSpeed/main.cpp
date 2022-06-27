/*! includes **/
#include <iostream>
#include <string>

#include "ScanerAPI/ScanerAPImessagesNetwork.h"
#include "ScanerAPI/scanerAPI_DLL_C.h"

/*! Print Datainterface information (name of the fields + type of the fileds) */
bool printInfoComMessage(DataInterface* dataInterface) {
  bool res = false;
  int nbField = Com_getFieldNumber(dataInterface);

  if (nbField == 0) return res;

  FieldInfo* fieldInfo = new FieldInfo[nbField];
  if (Com_getFieldInfoArray(dataInterface, fieldInfo)) {
    res = true;
    for (int i = 0; i < nbField; ++i) {
      std::cout << '\t' << fieldInfo[i].name << " type of " << fieldInfo[i].typeName << std::endl;
    }
  }
  delete[] fieldInfo;
  return res;
}

int main(int argc, char* argv[]) {
  /*! Process Com intialization **/
  if (Process_Init(argc, argv) == -1) {
    std::cout << "Error while initializing SCANeR API. Exiting" << std::endl;
    return 1;
  }

  /********************************************/
  /*! INITIALIZATION (Input / Output)        **/
  /********************************************/

  DataInterface* v = Com_declareInputData(NETWORK_IVEHICLE_VEHICLEUPDATE, 0);

  APIProcessState status = PS_DAEMON;

  /**************************************************************************/
  /*! MAIN LOOP                                                            **/
  /**************************************************************************/
  while (status != PS_DEAD) {
    /*! Process manager Run **/
    Process_Wait();
    Process_Run();

    /*! Process manager State **/
    APIProcessState oldStatus = status;
    status = Process_GetState();

    if (oldStatus != status) {
      switch (status) {
        case PS_DAEMON:
          std::cout << " SCANER API STATUS : DAEMON " << std::endl;
          break;
        case PS_DEAD:
          std::cout << " SCANER API STATUS : DEAD " << std::endl;
          break;
        case PS_LOADED:
          std::cout << " SCANER API STATUS : LOADED " << std::endl;
          break;
        case PS_PAUSED:
          std::cout << " SCANER API STATUS : PAUSED " << std::endl;
          break;
        case PS_READY:
          std::cout << " SCANER API STATUS : READY " << std::endl;
          break;
        case PS_RUNNING:
          std::cout << " SCANER API STATUS : RUNNING " << std::endl;
          break;
        default:
          break;
      }
    }

    /*! Scaner API is now running **/
    if (status == PS_RUNNING) {
      float speed[6];
      int r = Com_getFloatDataArray(v, "speed", speed, 6);
      std::cout << r << std::endl;
      std::cout << "speed: ";
      Com_updateInputs();
      for (int i = 0; i < 6; i++) {
        std::cout << speed[i] << ' ';
      }
      std::cout << std::endl;
    }
  }
  return 0;
}
