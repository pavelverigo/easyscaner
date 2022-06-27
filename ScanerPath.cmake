# This file contains pathes to SCANeR directories with build files, works for Windows only

# Find STUDIO directory in Environment variable, by default installation should be in "C:\AVSimulation"
set(STUDIO_DIR "$ENV{STUDIO_PATH}")
# message("DEBUG $ENV{STUDIO_PATH}")

if(NOT DEFINED STUDIO_DIR)
  message(FATAL_ERROR "CMake STUDIO_DIR environment varible is not found.")
endif()

# Below code checks for what SCANeR version is installed with giving priority for 1.9.
# Note below code can fail also in inner place of if statements, but this will mean that SCAnER installation was altered.
if(EXISTS "${STUDIO_DIR}/SCANeRstudio_1.9")
  # Support varial
  set(SCANER_API_DIR "${STUDIO_DIR}/SCANeRstudio_1.9/APIs")

  # Variables where lib and dll located
  set(SCANER_LIB_DIR "${SCANER_API_DIR}/lib/x64/vs2013")
  set(SCANER_DLL_DIR "${SCANER_API_DIR}/bin/x64/vs2013")
elseif(EXISTS "${STUDIO_DIR}/SCANeRstudio_2022")
  # Support variale
  set(SCANER_API_DIR "${STUDIO_DIR}/SCANeRstudio_2022/APIs")
  
  # Variables where lib and dll located
  set(SCANER_LIB_DIR "${SCANER_API_DIR}/lib/x64/vs2019")
  set(SCANER_DLL_DIR "${SCANER_API_DIR}/bin/x64/vs2019")
else()
  # If none above was true exit because we cannot compile
  message(FATAL_ERROR "Can not find SCANeR directory. DEBUG: STUDIO_DIR: ${STUDIO_DIR}")
endif()

# Include directory with all header files
set(SCANER_INCLUDE_DIR "${SCANER_API_DIR}/include")