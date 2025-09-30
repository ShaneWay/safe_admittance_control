cmake_minimum_required(VERSION 3.20)

project(kortex_admittance_control  LANGUAGES CXX)

# Activate C++ 11
set (CMAKE_CXX_STANDARD 11)
find_package(Eigen3)
message(${CMAKE_CXX_COMPILER})
set(CMAKE_INCLUDE_CURRENT_DIR   ON)
set(CMAKE_COLOR_MAKEFILE        ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


set(KORTEX_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include/kortex/")
set(RBDL_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include/rbdl/")

message(${PROJECT_SOURCE_DIR})



if(UNIX)
  add_definitions(-D_OS_UNIX)
elseif(WIN32)
  add_definitions(-D_OS_WINDOWS -DNOMINMAX)
  if(MSVC)
     add_compile_options(/bigobj)
  endif()
endif()

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 COMPONENTS NumPy)

link_libraries(${PROJECT_SOURCE_DIR}/lib/kortex_api/libKortexApiCpp.a)
link_libraries(pthread)
link_libraries(${PROJECT_SOURCE_DIR}/lib/rbdl/librbdl.so)
link_libraries(${PROJECT_SOURCE_DIR}/lib/rbdl/urdfreader/librbdl_urdfreader.so)

include_directories(${PROJECT_SOURCE_DIR}/include/kortex/utilities)
include_directories(${PROJECT_SOURCE_DIR}/controller)
include_directories(${PROJECT_SOURCE_DIR}/force_sensor)
include_directories(${PROJECT_SOURCE_DIR}/dynamics_model)
include_directories(${PROJECT_SOURCE_DIR}/kinematics_model)
include_directories(${PROJECT_SOURCE_DIR}/thirdParty/cxxopts)
include_directories(${PROJECT_SOURCE_DIR}/include/kortex)
include_directories(${PROJECT_SOURCE_DIR}/include/kortex/client)
include_directories(${PROJECT_SOURCE_DIR}/include/kortex/common)
include_directories(${PROJECT_SOURCE_DIR}/include/kortex/messages)
include_directories(${PROJECT_SOURCE_DIR}/include/kortex/client_stubs)
include_directories(${PROJECT_SOURCE_DIR}/include/kortex/message)
include_directories(${PROJECT_SOURCE_DIR}/include/kortex/thirdParty/include)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/rbdl)


include_directories(${EIGEN3_INCLUDE_DIR})


file(GLOB_RECURSE EXE_LIST RELATIVE ${PROJECT_SOURCE_DIR}/src "[0-9]*.cpp")

foreach ( SRC_FILE ${EXE_LIST} )

  string(REPLACE ".cpp" "" TARGET_EXE_NAME ${SRC_FILE})
  string(REPLACE "/" "_" TARGET_EXE_NAME ${TARGET_EXE_NAME})
  
  MESSAGE(${SRC_FILE})
  MESSAGE("creating TARGET_EXE_NAME: '${TARGET_EXE_NAME}'")

  add_executable(${TARGET_EXE_NAME} src/${SRC_FILE} 
  ./include/kortex/utilities/utilities.cpp 
  ./dynamics_model/dynamics_model.cpp 
  ./kinematics_model/kinematics_model.cpp 
  ./controller/controller_2.cpp 
  ./controller/controller.cpp 
  ./force_sensor/sriCommATParser.cpp
  ./force_sensor/sriCommATParser.h
  ./force_sensor/sriCommCircularBuffer.cpp
  ./force_sensor/sriCommCircularBuffer.h
  ./force_sensor/sriCommDefine.h
  ./force_sensor/sriCommManager.cpp
  ./force_sensor/sriCommManager.h
  ./force_sensor/sriCommParser.cpp
  ./force_sensor/sriCommParser.h
  ./force_sensor/sriCommTCPClient.cpp
  ./force_sensor/sriCommTCPClient.h
  ./force_sensor/sriCommM8218Parser.cpp
  ./force_sensor/sriCommM8218Parser.h
  )


  target_include_directories(${TARGET_EXE_NAME} PUBLIC ${Python3_INCLUDE_DIRS})
  target_include_directories(${TARGET_EXE_NAME} PUBLIC ${Python3_NumPy_INCLUDE_DIRS})

  message(${Python3_INCLUDE_DIRS})
  message(${Python3_LIBRARY_DIRS})
  message(${Python3_NumPy_INCLUDE_DIRS})

  target_link_libraries(${TARGET_EXE_NAME} PUBLIC
  Python3::Python
  Python3::Module
  )

  if(Python3_NumPy_FOUND)
    target_link_libraries(${TARGET_EXE_NAME} PUBLIC
      Python3::NumPy
      
    )
    message("have numpy")
    message(${Python3_NumPy_FOUND})
  else()
    target_compile_definitions(${TARGET_EXE_NAME} INTERFACE WITHOUT_NUMPY)
    message("no numpy")
  endif()
  
endforeach()
