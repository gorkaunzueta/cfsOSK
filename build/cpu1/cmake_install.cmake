# Install script for directory: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/cfe

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/exe")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "debug")
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
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cpu1/cf" TYPE FILE RENAME "cfe_es_startup.scr" FILES "/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/osk_defs/cpu1_cfe_es_startup.scr")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cpu1/cf" TYPE FILE RENAME "osk_to_pkt_tbl.json" FILES "/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/osk_defs/cpu1_osk_to_pkt_tbl.json")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cpu1/cf" TYPE FILE RENAME "osk_sch_msg_tbl.json" FILES "/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/osk_defs/cpu1_osk_sch_msg_tbl.json")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cpu1/cf" TYPE FILE RENAME "osk_sch_sch_tbl.json" FILES "/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/osk_defs/cpu1_osk_sch_sch_tbl.json")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cpu1/cf" TYPE FILE RENAME "osk_c_demo_tbl.json" FILES "/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/osk_defs/cpu1_osk_c_demo_tbl.json")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cpu1/cf" TYPE FILE RENAME "osk_c_demo_tbl.scanf" FILES "/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/osk_defs/cpu1_osk_c_demo_tbl.scanf")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cpu1/cf" TYPE FILE RENAME "osk_c_demo_tbl.xml" FILES "/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/osk_defs/cpu1_osk_c_demo_tbl.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cpu1/cf" TYPE FILE RENAME "osk_cpp_tbl.json" FILES "/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/osk_defs/cpu1_osk_cpp_tbl.json")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cpu1/cf" TYPE FILE RENAME "f42_ctrl_tbl.json" FILES "/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/osk_defs/cpu1_f42_ctrl_tbl.json")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cpu1/cf" TYPE FILE RENAME "isim_ini.json" FILES "/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/osk_defs/cpu1_isim_ini.json")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cpu1/cf" TYPE FILE RENAME "isim_tbl.json" FILES "/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/osk_defs/cpu1_isim_tbl.json")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cpu1/cf" TYPE FILE RENAME "filemgr_ini.json" FILES "/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/osk_defs/cpu1_filemgr_ini.json")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cpu1/cf" TYPE FILE RENAME "mqtt_ini.json" FILES "/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/osk_defs/cpu1_mqtt_ini.json")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cpu1/cf" TYPE FILE RENAME "scsim_ini.json" FILES "/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/osk_defs/cpu1_scsim_ini.json")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/cpu1/cf" TYPE FILE RENAME "scsim_tbl.json" FILES "/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/osk_defs/cpu1_scsim_tbl.json")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/osal/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/psp/pc-linux/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/cfs_lib/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/expat_lib/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_c_fw/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mqtt_lib/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/kit_ci/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/kit_sch/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/kit_to/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/cs/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/ds/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/fm/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/hs/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/lc/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/md/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/sc/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/hk/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/tftp/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/cf/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/i42/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/f42/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/isim/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/bm/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/hc/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/hsim/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/filemgr/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_c_demo/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_demo/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mqtt/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/imu_app/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/gps_app/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/bcm2835_lib/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mpu9dof_lib/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/gpsnodemcu_lib/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/cfe_core_default_cpu1/cmake_install.cmake")
  include("/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/cpu1/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
