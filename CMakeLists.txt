cmake_minimum_required(VERSION 2.8.3)
project(soem)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

OPTION (BUILD_SHARED_LIBS "Build shared libraries." OFF)

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
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${XENO_CFLAGS}")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS} -lnative")
endif()

set(HEADER_INSTALL_ROOT /usr/local/include/ CACHE PATH "Where to install the headers")
set(LIB_INSTALL_ROOT /usr/local/lib/ CACHE PATH "Where to install the libraries")

###########
## Build ##
###########
set(SOEM_LIB_NAME soem)
set(IIT_LIB_NAME iitecat_iface)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${soem_SOURCE_DIR}/include)
include_directories(./include)
include_directories(./include/soem-1.3.0) # not to break compilability in the soem sources

add_library(${SOEM_LIB_NAME}
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

target_link_libraries(${SOEM_LIB_NAME} pthread rt)


add_executable(simple_test test/simple_test.c)
add_executable(red_test test/red_test.c)
add_executable(firm_update test/firm_update.c)
add_executable(eepromtool test/eepromtool.c)
add_executable(ebox test/ebox.c)
add_executable(aliastool test/aliastool.c)
add_executable(slaveinfo test/slaveinfo.c)

target_link_libraries(simple_test ${SOEM_LIB_NAME})
target_link_libraries(red_test ${SOEM_LIB_NAME})
target_link_libraries(firm_update ${SOEM_LIB_NAME})
target_link_libraries(eepromtool ${SOEM_LIB_NAME})
target_link_libraries(ebox ${SOEM_LIB_NAME})
target_link_libraries(aliastool ${SOEM_LIB_NAME})
target_link_libraries(slaveinfo ${SOEM_LIB_NAME})


add_library(${IIT_LIB_NAME}
  ecat_iface/slave_wrapper.cpp
  ecat_iface/ec_master_iface.cpp
)	
target_link_libraries(${IIT_LIB_NAME} ${SOEM_LIB_NAME})

add_executable(ec_iface_firm_update ecat_iface/ec_iface_firm_update.cpp)
target_link_libraries(ec_iface_firm_update ${IIT_LIB_NAME})


# Install
install(DIRECTORY include/  DESTINATION ${HEADER_INSTALL_ROOT})
install(TARGETS ${SOEM_LIB_NAME} DESTINATION ${LIB_INSTALL_ROOT})
install(TARGETS ${IIT_LIB_NAME}  DESTINATION ${LIB_INSTALL_ROOT})
