cmake_minimum_required(VERSION 2.6.4)
project(CFS_MPU9DOF_LIB C)

include_directories(fsw/src)
include_directories(fsw/public_inc)

include_directories(${bcm2835_lib_MISSION_DIR}/fsw/public_inc)

# The shared OSAL and cFE include directories should always be used
# Note that this intentionally does NOT include PSP-specific includes, just the generic
include_directories(${CFECORE_SOURCE_DIR}/src/inc)
include_directories(${CFEPSP_SOURCE_DIR}/fsw/inc)

aux_source_directory(fsw/src LIB_SRC_FILES)

# Create the app module
add_cfe_app(mpu9dof_lib ${LIB_SRC_FILES})
target_link_libraries (mpu9dof_lib m)
