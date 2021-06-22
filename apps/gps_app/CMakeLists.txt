cmake_minimum_required(VERSION 2.6.4)
project(CFS_GPS_APP C)


include_directories(fsw/src)
include_directories(fsw/mission_inc)
include_directories(fsw/platform_inc)

include_directories(${bcm2835_lib_MISSION_DIR}/fsw/public_inc)
include_directories(${gpsnodemcu_lib_MISSION_DIR}/fsw/public_inc)

aux_source_directory(fsw/src APP_SRC_FILES)
aux_source_directory(fsw/tables APP_TABLE_FILES)

# Create the app module
add_cfe_app(gps_app ${APP_SRC_FILES})
target_link_libraries (gps_app m)

# Add table
add_cfe_tables(GpsAppTable fsw/tables/gps_app_tbl.c)
