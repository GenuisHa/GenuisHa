# Install script for directory: /home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/src/ucar_commander

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ucar_commander/srv" TYPE FILE FILES "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/src/ucar_commander/srv/nav_start.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ucar_commander/cmake" TYPE FILE FILES "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/build/ucar_commander/catkin_generated/installspace/ucar_commander-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/devel/include/ucar_commander")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/devel/share/roseus/ros/ucar_commander")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/devel/share/common-lisp/ros/ucar_commander")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/devel/share/gennodejs/ros/ucar_commander")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/devel/lib/python3/dist-packages/ucar_commander")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/devel/lib/python3/dist-packages/ucar_commander")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/build/ucar_commander/catkin_generated/installspace/ucar_commander.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ucar_commander/cmake" TYPE FILE FILES "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/build/ucar_commander/catkin_generated/installspace/ucar_commander-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ucar_commander/cmake" TYPE FILE FILES
    "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/build/ucar_commander/catkin_generated/installspace/ucar_commanderConfig.cmake"
    "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/build/ucar_commander/catkin_generated/installspace/ucar_commanderConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ucar_commander" TYPE FILE FILES "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/src/ucar_commander/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ucar_commander" TYPE PROGRAM FILES "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/build/ucar_commander/catkin_generated/installspace/ucar_commander_node.py")
endif()

