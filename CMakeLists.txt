cmake_minimum_required(VERSION 2.8.3)
project(soem)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroOptionalBuild)
include(MacroSetcap)

set(CMAKE_CXX_FLAGS     "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
set(ENABLE_XENO         CACHE BOOL "OFF")

if ( ${ENABLE_XENO} )
    find_package(Xenomai)
endif()

if ( Xenomai_FOUND )
    set(LIB_TYPE _rt)
else ()
    set(LIB_TYPE _nrt)
endif()



option(BUILD_SHARED_LIBS "Build shared libraries." ON)
set(HEADER_INSTALL_ROOT /usr/local/include/ CACHE PATH "Where to install the headers")
set(LIB_INSTALL_ROOT /usr/local/lib/ CACHE PATH "Where to install the libraries")

###########
## Build ##
###########
set(SOEM_LIB_NAME soem${LIB_TYPE})
set(IIT_LIB_NAME iitecat_iface${LIB_TYPE})

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
if ( Xenomai_FOUND )
    set_xeno_flags(${SOEM_LIB_NAME})
endif()
target_link_libraries(${SOEM_LIB_NAME} pthread rt)

add_executable(simple_test test/simple_test.c)
if ( Xenomai_FOUND )
    set_xeno_flags(simple_test)
endif()
target_link_libraries(simple_test ${SOEM_LIB_NAME})


add_library(${IIT_LIB_NAME}
  ecat_iface/slave_wrapper.cpp
  ecat_iface/ec_master_iface.cpp
)
if ( Xenomai_FOUND )
    set_xeno_flags(${IIT_LIB_NAME})
endif()
target_link_libraries(${IIT_LIB_NAME} ${SOEM_LIB_NAME})

add_executable(ec_iface_firm_update ecat_iface/ec_iface_firm_update.cpp)
if ( Xenomai_FOUND )
    set_xeno_flags(ec_iface_firm_update)
endif()
target_link_libraries(ec_iface_firm_update ${IIT_LIB_NAME})

add_executable(ec_iface_foe_test ecat_iface/ec_iface_foe_test.cpp)
if ( Xenomai_FOUND )
    set_xeno_flags(ec_iface_foe_test)
endif()
target_link_libraries(ec_iface_foe_test ${IIT_LIB_NAME})


# Install
install(DIRECTORY include/  DESTINATION ${HEADER_INSTALL_ROOT})
install(TARGETS ${SOEM_LIB_NAME} DESTINATION ${LIB_INSTALL_ROOT})
install(TARGETS ${IIT_LIB_NAME}  DESTINATION ${LIB_INSTALL_ROOT})
