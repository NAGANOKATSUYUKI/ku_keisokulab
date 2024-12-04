execute_process(COMMAND "/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_mounted_devices_samples/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/keisoku/catkin_ws/build/hsrb_samples/hsrb_mounted_devices_samples/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
