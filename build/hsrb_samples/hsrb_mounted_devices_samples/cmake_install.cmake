# Install script for directory: /home/keisoku/catkin_ws/src/hsrb_samples/hsrb_mounted_devices_samples

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/keisoku/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_mounted_devices_samples/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_mounted_devices_samples/catkin_generated/installspace/hsrb_mounted_devices_samples.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hsrb_mounted_devices_samples/cmake" TYPE FILE FILES
    "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_mounted_devices_samples/catkin_generated/installspace/hsrb_mounted_devices_samplesConfig.cmake"
    "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_mounted_devices_samples/catkin_generated/installspace/hsrb_mounted_devices_samplesConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hsrb_mounted_devices_samples" TYPE FILE FILES "/home/keisoku/catkin_ws/src/hsrb_samples/hsrb_mounted_devices_samples/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hsrb_mounted_devices_samples" TYPE PROGRAM FILES "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_mounted_devices_samples/catkin_generated/installspace/speak_object_weight.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hsrb_mounted_devices_samples" TYPE PROGRAM FILES "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_mounted_devices_samples/catkin_generated/installspace/light_switch_led.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hsrb_mounted_devices_samples" TYPE PROGRAM FILES "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_mounted_devices_samples/catkin_generated/installspace/change_led_color.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hsrb_mounted_devices_samples" TYPE PROGRAM FILES "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_mounted_devices_samples/catkin_generated/installspace/suction_controller.py")
endif()

