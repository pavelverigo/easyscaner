cmake_minimum_required(VERSION 3.14)
project(easyscaner)

set(CMAKE_CXX_STANDARD 17)

include(FetchContent)

# Compile executables only in Windows
if(WIN32)
  # Add SCANeR directories and their pathes to variables
  include(ScanerPath.cmake)

  # Find .lib file
  find_library(SCANER_LIB SCANeR_API_C PATHS "${SCANER_LIB_DIR}")

  # Add samples
  add_executable(getSpeed "samples/ScanerAPI/getSpeed/main.cpp")
  target_include_directories(getSpeed PUBLIC "${SCANER_INCLUDE_DIR}")
  target_link_libraries(getSpeed PRIVATE "${SCANER_LIB}")
      
  # Add EasyScaner as library
  add_library(EasyScaner STATIC
    "EasyScaner/Scaner.cpp"
    "EasyScaner/Vehicle.cpp"
  )
  target_include_directories(EasyScaner PUBLIC "${SCANER_INCLUDE_DIR}")
  target_link_libraries(EasyScaner PRIVATE "${SCANER_LIB}")

  # Add Python
  add_subdirectory(python)

  # Add Python samples with EasyScaner
  add_executable(helloWorld "samples/EasyScaner/HelloWorld.cpp")
  target_include_directories(helloWorld PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(helloWorld EasyScaner)

  add_executable(decelerate "samples/EasyScaner/Decelerate.cpp")
  target_include_directories(decelerate PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(decelerate EasyScaner)

  add_executable(IDM+ "samples/EasyScaner/IDM+.cpp")
  target_include_directories(IDM+ PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(IDM+ EasyScaner)

  add_executable(IDM+Sensors "samples/EasyScaner/IDM+ with sensors.cpp")
  target_include_directories(IDM+Sensors PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(IDM+Sensors EasyScaner)

  add_executable(ChangeLane "samples/EasyScaner/ChangeLane.cpp")
  target_include_directories(ChangeLane PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(ChangeLane EasyScaner)

  add_executable(mobil "samples/EasyScaner/MOBIL.cpp")
  target_include_directories(mobil PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(mobil EasyScaner)
endif()

# If we in Docker we test
if(UNIX)
  # Compile EasyScaner to object file, because we will not able to link .lib file in Linux
  add_library(EasyScaner OBJECT
    "EasyScaner/Scaner.cpp"
    "EasyScaner/Vehicle.cpp"
  )
  target_include_directories(EasyScaner PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/test/include")


  # Install and use Google Test
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
  )
  # For Windows: Prevent overriding the parent project's compiler/linker settings
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()

  # Add test.cpp to test executable
  add_executable(mytest test/test.cpp)
  target_link_libraries(mytest gtest_main)

  include(GoogleTest)
  gtest_discover_tests(mytest)
endif()
