# Install script for directory: /home/student/myros/src/dobot

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/student/myros/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot/srv" TYPE FILE FILES
    "/home/student/myros/src/dobot/srv/SetCmdTimeout.srv"
    "/home/student/myros/src/dobot/srv/GetDeviceSN.srv"
    "/home/student/myros/src/dobot/srv/SetDeviceName.srv"
    "/home/student/myros/src/dobot/srv/GetDeviceName.srv"
    "/home/student/myros/src/dobot/srv/GetDeviceVersion.srv"
    "/home/student/myros/src/dobot/srv/GetPose.srv"
    "/home/student/myros/src/dobot/srv/GetAlarmsState.srv"
    "/home/student/myros/src/dobot/srv/ClearAllAlarmsState.srv"
    "/home/student/myros/src/dobot/srv/SetHOMEParams.srv"
    "/home/student/myros/src/dobot/srv/GetHOMEParams.srv"
    "/home/student/myros/src/dobot/srv/SetHOMECmd.srv"
    "/home/student/myros/src/dobot/srv/SetEndEffectorParams.srv"
    "/home/student/myros/src/dobot/srv/GetEndEffectorParams.srv"
    "/home/student/myros/src/dobot/srv/SetEndEffectorLaser.srv"
    "/home/student/myros/src/dobot/srv/GetEndEffectorLaser.srv"
    "/home/student/myros/src/dobot/srv/SetEndEffectorSuctionCup.srv"
    "/home/student/myros/src/dobot/srv/GetEndEffectorSuctionCup.srv"
    "/home/student/myros/src/dobot/srv/SetEndEffectorGripper.srv"
    "/home/student/myros/src/dobot/srv/GetEndEffectorGripper.srv"
    "/home/student/myros/src/dobot/srv/SetJOGJointParams.srv"
    "/home/student/myros/src/dobot/srv/GetJOGJointParams.srv"
    "/home/student/myros/src/dobot/srv/SetJOGCoordinateParams.srv"
    "/home/student/myros/src/dobot/srv/GetJOGCoordinateParams.srv"
    "/home/student/myros/src/dobot/srv/SetJOGCommonParams.srv"
    "/home/student/myros/src/dobot/srv/GetJOGCommonParams.srv"
    "/home/student/myros/src/dobot/srv/SetJOGCmd.srv"
    "/home/student/myros/src/dobot/srv/SetPTPJointParams.srv"
    "/home/student/myros/src/dobot/srv/GetPTPJointParams.srv"
    "/home/student/myros/src/dobot/srv/SetPTPCoordinateParams.srv"
    "/home/student/myros/src/dobot/srv/GetPTPCoordinateParams.srv"
    "/home/student/myros/src/dobot/srv/SetPTPJumpParams.srv"
    "/home/student/myros/src/dobot/srv/GetPTPJumpParams.srv"
    "/home/student/myros/src/dobot/srv/SetPTPCommonParams.srv"
    "/home/student/myros/src/dobot/srv/GetPTPCommonParams.srv"
    "/home/student/myros/src/dobot/srv/SetPTPCmd.srv"
    "/home/student/myros/src/dobot/srv/SetCPParams.srv"
    "/home/student/myros/src/dobot/srv/GetCPParams.srv"
    "/home/student/myros/src/dobot/srv/SetCPCmd.srv"
    "/home/student/myros/src/dobot/srv/SetARCParams.srv"
    "/home/student/myros/src/dobot/srv/GetARCParams.srv"
    "/home/student/myros/src/dobot/srv/SetARCCmd.srv"
    "/home/student/myros/src/dobot/srv/SetWAITCmd.srv"
    "/home/student/myros/src/dobot/srv/SetTRIGCmd.srv"
    "/home/student/myros/src/dobot/srv/SetIOMultiplexing.srv"
    "/home/student/myros/src/dobot/srv/GetIOMultiplexing.srv"
    "/home/student/myros/src/dobot/srv/SetIODO.srv"
    "/home/student/myros/src/dobot/srv/GetIODO.srv"
    "/home/student/myros/src/dobot/srv/SetIOPWM.srv"
    "/home/student/myros/src/dobot/srv/GetIOPWM.srv"
    "/home/student/myros/src/dobot/srv/GetIODI.srv"
    "/home/student/myros/src/dobot/srv/GetIOADC.srv"
    "/home/student/myros/src/dobot/srv/SetEMotor.srv"
    "/home/student/myros/src/dobot/srv/GetInfraredSensor.srv"
    "/home/student/myros/src/dobot/srv/SetInfraredSensor.srv"
    "/home/student/myros/src/dobot/srv/SetColorSensor.srv"
    "/home/student/myros/src/dobot/srv/GetColorSensor.srv"
    "/home/student/myros/src/dobot/srv/SetQueuedCmdStartExec.srv"
    "/home/student/myros/src/dobot/srv/SetQueuedCmdStopExec.srv"
    "/home/student/myros/src/dobot/srv/SetQueuedCmdForceStopExec.srv"
    "/home/student/myros/src/dobot/srv/SetQueuedCmdClear.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot/cmake" TYPE FILE FILES "/home/student/myros/build/dobot/catkin_generated/installspace/dobot-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/student/myros/devel/include/dobot")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/student/myros/devel/share/roseus/ros/dobot")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/student/myros/devel/share/common-lisp/ros/dobot")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/student/myros/devel/share/gennodejs/ros/dobot")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/student/myros/devel/lib/python2.7/dist-packages/dobot")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/student/myros/devel/lib/python2.7/dist-packages/dobot")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/student/myros/build/dobot/catkin_generated/installspace/dobot.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot/cmake" TYPE FILE FILES "/home/student/myros/build/dobot/catkin_generated/installspace/dobot-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot/cmake" TYPE FILE FILES
    "/home/student/myros/build/dobot/catkin_generated/installspace/dobotConfig.cmake"
    "/home/student/myros/build/dobot/catkin_generated/installspace/dobotConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot" TYPE FILE FILES "/home/student/myros/src/dobot/package.xml")
endif()

