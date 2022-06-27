#pragma once

// This header purpose to fix the problem that "ScanerAPI/ScanerAPImessagesNetwork.h" does not have header guards.
// Also this header can be "called" starting point of mocking ScanerAPI

#include "ScanerAPI/ScanerAPImessagesNetwork.h"
#include "ScanerAPI/scanerAPI_DLL_C.h"

// Fix issue with SCANeR 2022 not having this define
#define NETWORK_ISENSOR_ROADSENSORDETECTEDPOINTS "Network/ISensor/RoadSensorDetectedPoints"
