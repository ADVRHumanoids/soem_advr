cmake_minimum_required(VERSION 2.8.3)
project(soem)


#find_path(RTNET_INCLUDE_DIR NAMES rtnet.h HINTS /usr/local/rtnet/include )
find_path(RTNET_INCLUDE_DIR NAMES rtnet.h PATH_SUFFIXES rtnet/include)
#set(RTNET_INCLUDE_DIR "/usr/local/rtnet/include")
set(ENABLE_RTNET  ON)

if(ENABLE_RTNET AND RTNET_INCLUDE_DIR)
  message("Found RTNET: ${RTNET_INCLUDE_DIR}")
  include_directories(${RTNET_INCLUDE_DIR})
  # try Xenomai
  execute_process(COMMAND xeno-config --skin=posix --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND xeno-config --skin=posix --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${XENO_CFLAGS}")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS} -lnative")
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${soem_SOURCE_DIR}/include)

add_library(soem	
  src/osal.c
  src/oshw.c
  src/nicdrv.c
  src/ethercatsoe.c
  src/ethercatprint.c
  src/ethercatmain.c
  src/ethercatfoe.c
  src/ethercatdc.c
  src/ethercatconfig.c
  src/ethercatcoe.c
  src/ethercatbase.c
)

target_link_libraries(soem pthread rt)


add_executable(simple_test test/simple_test.c)
add_executable(red_test test/red_test.c)
add_executable(firm_update test/firm_update.c)
add_executable(eepromtool test/eepromtool.c)
add_executable(ebox test/ebox.c)
add_executable(aliastool test/aliastool.c)
add_executable(slaveinfo test/slaveinfo.c)

target_link_libraries(simple_test soem)
target_link_libraries(red_test soem)
target_link_libraries(firm_update soem)
target_link_libraries(eepromtool soem)
target_link_libraries(ebox soem)
target_link_libraries(aliastool soem)
target_link_libraries(slaveinfo soem)


add_library(ecat_iface
  ecat_iface/ec_master_iface.cpp
  #ecat_iface/ec_slave.cpp
)	


target_link_libraries(ecat_iface soem)

add_executable(xeno_test ecat_iface/xeno_test.cpp)

target_link_libraries(xeno_test ecat_iface)

