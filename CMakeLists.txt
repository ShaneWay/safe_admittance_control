cmake_minimum_required(VERSION 3.20)

project(kortex_admittance_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pinocchio REQUIRED)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 COMPONENTS NumPy)

message("CXX compiler: ${CMAKE_CXX_COMPILER}")
message("Project source dir: ${PROJECT_SOURCE_DIR}")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_COLOR_MAKEFILE ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(UNIX)
    add_definitions(-D_OS_UNIX)
elseif(WIN32)
    add_definitions(-D_OS_WINDOWS -DNOMINMAX)
    if(MSVC)
        add_compile_options(/bigobj)
    endif()
endif()

# ------------------------------------------------------------
# Include directories
# ------------------------------------------------------------
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/kortex
    ${PROJECT_SOURCE_DIR}/include/kortex/utilities
    ${PROJECT_SOURCE_DIR}/include/kortex/client
    ${PROJECT_SOURCE_DIR}/include/kortex/common
    ${PROJECT_SOURCE_DIR}/include/kortex/messages
    ${PROJECT_SOURCE_DIR}/include/kortex/client_stubs
    ${PROJECT_SOURCE_DIR}/include/kortex/message
    ${PROJECT_SOURCE_DIR}/include/kortex/thirdParty/include

    ${PROJECT_SOURCE_DIR}/controller
    ${PROJECT_SOURCE_DIR}/robot_control
    ${PROJECT_SOURCE_DIR}/force_sensor
    ${PROJECT_SOURCE_DIR}/dynamics_model
    ${PROJECT_SOURCE_DIR}/kinematics_model
    ${PROJECT_SOURCE_DIR}/thirdParty/cxxopts

    ${EIGEN3_INCLUDE_DIR}
)

# ------------------------------------------------------------
# Common libraries
# ------------------------------------------------------------
link_libraries(
    ${PROJECT_SOURCE_DIR}/lib/kortex_api/libKortexApiCpp.a
    pthread
)

# ------------------------------------------------------------
# Search executable source files
# ------------------------------------------------------------
file(GLOB_RECURSE EXE_LIST RELATIVE ${PROJECT_SOURCE_DIR}/src
    "1*.cpp"
    "2*.cpp"
    "3*.cpp"
    "4*.cpp"
    "5*.cpp"
)

# ------------------------------------------------------------
# Source groups
# ------------------------------------------------------------

# Main torque/admittance control executable sources
set(FULL_SOURCES
    ./include/kortex/utilities/utilities.cpp
    ./kinematics_model/kinematics_model.cpp

    ./controller/ConfigLoader.cpp
    ./controller/BaseController.cpp

    ./robot_control/SafeAdmittanceExperiment.cpp
    ./robot_control/KortexRobotInterface.cpp

    ./force_sensor/sriCommATParser.cpp
    ./force_sensor/sriCommCircularBuffer.cpp
    ./force_sensor/sriCommManager.cpp
    ./force_sensor/sriCommParser.cpp
    ./force_sensor/sriCommTCPClient.cpp
    ./force_sensor/sriCommM8218Parser.cpp
)

# Move angular / Kortex utility examples
set(UTILITIES_ONLY_SOURCES
    ./include/kortex/utilities/utilities.cpp
)

# Forward kinematics executable
set(KINEMATICS_ONLY_SOURCES
    ./kinematics_model/kinematics_model.cpp
)

# Force sensor test executable
set(FORCE_SENSOR_ONLY_SOURCES
    ./force_sensor/sriCommATParser.cpp
    ./force_sensor/sriCommCircularBuffer.cpp
    ./force_sensor/sriCommManager.cpp
    ./force_sensor/sriCommParser.cpp
    ./force_sensor/sriCommTCPClient.cpp
    ./force_sensor/sriCommM8218Parser.cpp
)

# ------------------------------------------------------------
# Generate executables
# ------------------------------------------------------------
foreach(SRC_FILE ${EXE_LIST})

    string(REPLACE ".cpp" "" TARGET_EXE_NAME ${SRC_FILE})
    string(REPLACE "/" "_" TARGET_EXE_NAME ${TARGET_EXE_NAME})

    message("SRC_FILE: ${SRC_FILE}")
    message("creating TARGET_EXE_NAME: ${TARGET_EXE_NAME}")

    if(SRC_FILE MATCHES "^1.*\\.cpp$")
        set(EXTRA_SOURCES ${FULL_SOURCES})

    elseif(SRC_FILE MATCHES "^5.*\\.cpp$")
        set(EXTRA_SOURCES ${FORCE_SENSOR_ONLY_SOURCES})

    elseif(SRC_FILE MATCHES "^[23].*\\.cpp$")
        set(EXTRA_SOURCES ${UTILITIES_ONLY_SOURCES})

    elseif(SRC_FILE MATCHES "^4.*\\.cpp$")
        set(EXTRA_SOURCES ${KINEMATICS_ONLY_SOURCES})

    else()
        set(EXTRA_SOURCES "")
    endif()

    add_executable(${TARGET_EXE_NAME}
        src/${SRC_FILE}
        ${EXTRA_SOURCES}
    )

    target_include_directories(${TARGET_EXE_NAME} PUBLIC
        ${Python3_INCLUDE_DIRS}
        ${Python3_NumPy_INCLUDE_DIRS}
    )

    target_link_libraries(${TARGET_EXE_NAME} PUBLIC
        yaml-cpp
        stdc++fs
    )

    # Python is only needed by targets that use matplotlibcpp.
    # 1: main controller with DataLogger plots
    # 4: forward kinematics may plot trajectories
    if(SRC_FILE MATCHES "^[14].*\\.cpp$")
        target_link_libraries(${TARGET_EXE_NAME} PUBLIC
            Python3::Python
            Python3::Module
        )

        if(Python3_NumPy_FOUND)
            target_link_libraries(${TARGET_EXE_NAME} PUBLIC Python3::NumPy)
        else()
            target_compile_definitions(${TARGET_EXE_NAME} PUBLIC WITHOUT_NUMPY)
        endif()
    endif()

    # Pinocchio is needed by robot control / kinematics related targets.
    if(SRC_FILE MATCHES "^[14].*\\.cpp$")
        target_link_libraries(${TARGET_EXE_NAME} PUBLIC
            pinocchio::pinocchio
        )
        target_compile_definitions(${TARGET_EXE_NAME} PUBLIC USE_PINOCCHIO)
    endif()

endforeach()