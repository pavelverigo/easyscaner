cmake_minimum_required(VERSION 3.12)
project(pyscaner)

# Download pybind11 from github
FetchContent_Declare(
  pybind11
  URL https://github.com/pybind/pybind11/archive/refs/tags/v2.9.2.zip
)
FetchContent_MakeAvailable(pybind11)

# Create module
pybind11_add_module(pyscaner "src/main.cpp")
target_link_libraries(pyscaner PRIVATE EasyScaner)
target_include_directories(pyscaner PUBLIC ${CMAKE_SOURCE_DIR})

# Copying dll
file(GLOB SCANER_DLL "${SCANER_DLL_DIR}/*.dll")
add_custom_command(TARGET pyscaner POST_BUILD
COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SCANER_DLL} $<TARGET_FILE_DIR:pyscaner>
COMMENT "Copying SCANeR dll files."
COMMAND_EXPAND_LISTS)

# Copying setup.py that contains info about install python package
add_custom_command(TARGET pyscaner POST_BUILD
COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/setup.py" "${CMAKE_CURRENT_SOURCE_DIR}/MANIFEST.in" $<TARGET_FILE_DIR:pyscaner>
COMMENT "Copying setup.py.")

# We need python for next command
find_package(Python)
# Using "python -m pip install -e PATH" for installing package globally, "pip list" should show it
add_custom_command(TARGET pyscaner POST_BUILD
COMMAND ${Python_EXECUTABLE} -m pip install -e $<TARGET_FILE_DIR:pyscaner> # Installing in -e or --editorial mode, not ideal for final installation
COMMENT "Executing pip install.")
