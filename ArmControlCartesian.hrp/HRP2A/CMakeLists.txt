cmake_minimum_required(VERSION 2.6)

set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)
if (NOT CMAKE_MODULE_PATH)
  set(CMAKE_MODULE_PATH /opt/grx/share/hrpsys/cmake_modules/ CACHE STRING "set path to cmake_modules")
endif (NOT CMAKE_MODULE_PATH)

####
## settings for OpenRTM and OpenHRP
##
include(FindPkgConfig)
include(CheckIncludeFiles)
find_package(OpenRTM REQUIRED)
find_package(OpenHRP REQUIRED)
include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})
include_directories(${OPENHRP_INCLUDE_DIRS})
link_directories(${OPENHRP_LIBRARY_DIRS})
include_directories(/opt/grx/include)
include_directories(${CMAKE_MODULE_PATH})

if(QNXNTO)
  include_directories(/usr/pkg/include)
  set(NO_COMP true)
  set(CPACK_GENERATOR "DEB")
  set(CPACK_DEB_COMPONENT_INSTALL ON)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(CPACK_SYSTEM_NAME "amd64")
  else(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(CPACK_SYSTEM_NAME "i386")
  endif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(CPACK_PACKAGE_NAME "UserComponent")
  set(CPACK_DEBIAN_PACKAGE_MAINTAINTER "GRX Support")
  set(CPACK_PACKAGE_CONTACT "support@generalrobotix.com")
  set(CPACK_PACKAGE_VENDOR "General Robotix, Inc.")
  #set(CPACK_PACKAGE_INSTALL_DIRECTORY ${CMAKE_INSTALL_PREFIX})
  #set(CPACK_INSTALL_DIRECTORIES ${CMAKE_INSTALL_PREFIX})
  #set(CPACK_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
  set(CPACK_DEBIAN_BUILD_DEPENDS g++)
  include(CPack)
  include(UploadPPA)
endif(QNXNTO)

include(GenerateStubSkel)

####
## for each component
##
include_directories(../interpolator)

set(comp_name ${COMP_NAME})
set(comp_sources ${comp_name}.cpp)

generate_stub_skel(${comp_name}Service)
set(comp_sources ${comp_sources} ${comp_name}Service_impl.cpp ${comp_name}ServiceSk.cpp)


####
## for nextage
##
#set(comp_sources ${comp_sources} ${CMAKE_MODULE_PATH}HrprtcBase.cpp)
#generate_hrpsys_stub_skel_wrapper(JointDataTypes)
#set(comp_sources ${comp_sources} JointDataTypesStub.cpp)


# interpolator
set(comp_sources ${comp_sources} ../interpolator/Interpolator.cpp ../interpolator/RotationalInterpolator.cpp)


add_library(${comp_name} SHARED ${comp_sources})
target_link_libraries(${comp_name} ${OPENRTM_LIBRARIES} hrpModel-3.1 hrpUtil-3.1)
target_link_libraries(${comp_name} lapack blas f2c m)
set_target_properties(${comp_name} PROPERTIES PREFIX "")

if (NO_COMP)
  set(target ${comp_name})
else()
  set(target ${comp_name} ${comp_name}Comp)
  add_executable(${comp_name}Comp ${comp_name}Comp.cpp ${comp_sources})
  target_link_libraries(${comp_name}Comp ${OPENRTM_LIBRARIES} hrpModel-3.1 hrpUtil-3.1)
  target_link_libraries(${comp_name}Comp boost_filesystem)
  target_link_libraries(${comp_name}Comp m)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb3 -fno-schedule-insns -fno-schedule-insns2 -fno-strict-aliasing")

#install(TARGETS ${target}
#  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
#  )
install(TARGETS ${comp_name}
  LIBRARY DESTINATION ${ROBOT_NAME}/lib CONFIGURATIONS Release Debug
  )
