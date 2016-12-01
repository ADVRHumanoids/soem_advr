cmake_minimum_required(VERSION 2.8.3)
project(soem)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroOptionalBuild)
include(MacroSetcap)

set(CMAKE_CXX_FLAGS     "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
set(ENABLE_XENO         CACHE BOOL "OFF")

if ( ${ENABLE_XENO} )
    find_package(Xenomai)
    find_package(RTnet)
endif()

if ( Xenomai_FOUND )
    set(LIB_TYPE _rt)
else ()
    set(LIB_TYPE _nrt)
endif()



option(BUILD_SHARED_LIBS "Build shared libraries." ON)

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

#add_executable(simple_test test/simple_test.c)
#target_link_libraries(simple_test ${SOEM_LIB_NAME})


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

########################################################################

set(VARS_PREFIX ${SOEM_LIB_NAME})

set(${SOEM_LIB_NAME}_MAJOR_VERSION 0)
set(${SOEM_LIB_NAME}_MINOR_VERSION 0)
set(${SOEM_LIB_NAME}_PATCH_VERSION 1)
set(${SOEM_LIB_NAME}_VERSION ${${SOEM_LIB_NAME}_MAJOR_VERSION}.${${SOEM_LIB_NAME}_MINOR_VERSION}.${${SOEM_LIB_NAME}_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(${SOEM_LIB_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(${SOEM_LIB_NAME} PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                               SOVERSION ${${VARS_PREFIX}_VERSION})

install(DIRECTORY include/
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

install(TARGETS ${SOEM_LIB_NAME}
        EXPORT ${SOEM_LIB_NAME}
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)



#enabling it will add all XBotInterface dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${SOEM_LIB_NAME})


include(InstallBasicPackageFiles)
install_basic_package_files(${SOEM_LIB_NAME} VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)
                                    
set(VARS_PREFIX ${IIT_LIB_NAME})

set(${IIT_LIB_NAME}_MAJOR_VERSION 0)
set(${IIT_LIB_NAME}_MINOR_VERSION 0)
set(${IIT_LIB_NAME}_PATCH_VERSION 1)
set(${IIT_LIB_NAME}_VERSION ${${IIT_LIB_NAME}_MAJOR_VERSION}.${${IIT_LIB_NAME}_MINOR_VERSION}.${${IIT_LIB_NAME}_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(${IIT_LIB_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(${IIT_LIB_NAME} PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                               SOVERSION ${${VARS_PREFIX}_VERSION})

install(DIRECTORY include/
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

install(TARGETS ${IIT_LIB_NAME}
        EXPORT ${IIT_LIB_NAME}
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)



#enabling it will add all XBotInterface dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${IIT_LIB_NAME})


include(InstallBasicPackageFiles)
install_basic_package_files(${IIT_LIB_NAME} VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)                                    

include(AddUninstallTarget)
