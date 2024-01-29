# Install script for directory: /home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/src/mecanum_sim/nexus_4wd_mecanum_gazebo

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/build/mecanum_sim/nexus_4wd_mecanum_gazebo/catkin_generated/installspace/nexus_4wd_mecanum_gazebo.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nexus_4wd_mecanum_gazebo/cmake" TYPE FILE FILES
    "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/build/mecanum_sim/nexus_4wd_mecanum_gazebo/catkin_generated/installspace/nexus_4wd_mecanum_gazeboConfig.cmake"
    "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/build/mecanum_sim/nexus_4wd_mecanum_gazebo/catkin_generated/installspace/nexus_4wd_mecanum_gazeboConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nexus_4wd_mecanum_gazebo" TYPE FILE FILES "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/src/mecanum_sim/nexus_4wd_mecanum_gazebo/package.xml")
endif()

