# Install script for directory: /home/keisoku/catkin_ws/src/hsrb_samples/hsrb_motion_samples

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_motion_samples/catkin_generated/installspace/hsrb_motion_samples.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hsrb_motion_samples/cmake" TYPE FILE FILES
    "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_motion_samples/catkin_generated/installspace/hsrb_motion_samplesConfig.cmake"
    "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_motion_samples/catkin_generated/installspace/hsrb_motion_samplesConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hsrb_motion_samples" TYPE FILE FILES "/home/keisoku/catkin_ws/src/hsrb_samples/hsrb_motion_samples/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples" TYPE PROGRAM FILES "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_motion_samples/catkin_generated/installspace/arm_actionlib.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples" TYPE PROGRAM FILES "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_motion_samples/catkin_generated/installspace/arm_actionlib_cancel.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples" TYPE PROGRAM FILES "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_motion_samples/catkin_generated/installspace/arm_message.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples" TYPE PROGRAM FILES "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_motion_samples/catkin_generated/installspace/arm_message_cancel.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples" TYPE PROGRAM FILES "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_motion_samples/catkin_generated/installspace/gripper_actionlib.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples" TYPE PROGRAM FILES "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_motion_samples/catkin_generated/installspace/gripper_actionlib_grasp.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples" TYPE PROGRAM FILES "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_motion_samples/catkin_generated/installspace/gripper_actionlib_trajectory.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples" TYPE PROGRAM FILES "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_motion_samples/catkin_generated/installspace/gripper_message.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples" TYPE PROGRAM FILES "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_motion_samples/catkin_generated/installspace/head_actionlib.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples" TYPE PROGRAM FILES "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_motion_samples/catkin_generated/installspace/head_message.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples" TYPE PROGRAM FILES "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_motion_samples/catkin_generated/installspace/joint_impedance_client.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples" TYPE PROGRAM FILES "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_motion_samples/catkin_generated/installspace/navigation_actionlib.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples" TYPE PROGRAM FILES "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_motion_samples/catkin_generated/installspace/navigation_message.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples" TYPE PROGRAM FILES "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_motion_samples/catkin_generated/installspace/omni_actionlib.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples" TYPE PROGRAM FILES "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_motion_samples/catkin_generated/installspace/omni_message.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples" TYPE PROGRAM FILES "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_motion_samples/catkin_generated/installspace/omni_message_velocity.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/arm_message" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/arm_message")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/arm_message"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples" TYPE EXECUTABLE FILES "/home/keisoku/catkin_ws/devel/lib/hsrb_motion_samples/arm_message")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/arm_message" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/arm_message")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/arm_message"
         OLD_RPATH "/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/arm_message")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/arm_actionlib" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/arm_actionlib")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/arm_actionlib"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples" TYPE EXECUTABLE FILES "/home/keisoku/catkin_ws/devel/lib/hsrb_motion_samples/arm_actionlib")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/arm_actionlib" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/arm_actionlib")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/arm_actionlib"
         OLD_RPATH "/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/arm_actionlib")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/head_message" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/head_message")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/head_message"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples" TYPE EXECUTABLE FILES "/home/keisoku/catkin_ws/devel/lib/hsrb_motion_samples/head_message")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/head_message" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/head_message")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/head_message"
         OLD_RPATH "/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/head_message")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/head_actionlib" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/head_actionlib")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/head_actionlib"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples" TYPE EXECUTABLE FILES "/home/keisoku/catkin_ws/devel/lib/hsrb_motion_samples/head_actionlib")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/head_actionlib" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/head_actionlib")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/head_actionlib"
         OLD_RPATH "/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples/head_actionlib")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hsrb_motion_samples" TYPE DIRECTORY FILES "/home/keisoku/catkin_ws/src/hsrb_samples/hsrb_motion_samples/tests/" USE_SOURCE_PERMISSIONS FILES_MATCHING REGEX "/[^/]*\\.py$")
endif()

