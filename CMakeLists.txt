cmake_minimum_required(VERSION 3.12)

set(CMAKE_WARN_DEVELOPER OFF)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_C_STANDARD 99 CACHE STRING "C standard" FORCE)
set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ standard" FORCE)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(phybot_software_sdk)
add_compile_options(-Wno-return-type)

set(JOYSTICK_TYPE "Model_airplane" CACHE STRING "Joystick type: 'Joystick' or 'Model_airplane'")
set_property(CACHE JOYSTICK_TYPE PROPERTY STRINGS "Joystick" "Model_airplane")
set(IMU_TYPE "Hipnuc" CACHE STRING "IMU type: Hipnuc (old) or IMU (new)")
set_property(CACHE IMU_TYPE PROPERTY STRINGS "Hipnuc" "IMU")
if(IMU_TYPE STREQUAL "Hipnuc")
    add_compile_definitions(DEVICE_IMU_HIPNUC)
elseif(IMU_TYPE STREQUAL "IMU")
    add_compile_definitions(DEVICE_IMU_CUSTOM)
else()
    message(FATAL_ERROR "Invalid IMU_TYPE: ${IMU_TYPE}. Use Hipnuc or IMU")
endif()

set(CMAKE_PREFIX_PATH "/opt/ros/noetic" ${PROJECT_SOURCE_DIR}/ThirdParty/libtorch/share/cmake/Torch)
find_package(Torch REQUIRED)

if(JOYSTICK_TYPE STREQUAL "Joystick")
    set(PHYBOT_JOYSTICK_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/Joystick/include")
elseif(JOYSTICK_TYPE STREQUAL "Model_airplane")
    set(PHYBOT_JOYSTICK_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/Model_airplane_Joystick/include")
else()
    message(FATAL_ERROR "Invalid JOYSTICK_TYPE: ${JOYSTICK_TYPE}")
endif()

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/ThirdParty
    ${PROJECT_SOURCE_DIR}/ThirdParty/eigen3
    ${PROJECT_SOURCE_DIR}/ThirdParty/libtorch/include
    ${PROJECT_SOURCE_DIR}/ThirdParty/boost/include
    ${PROJECT_SOURCE_DIR}/ThirdParty/yaml/include
    ${PROJECT_SOURCE_DIR}/ThirdParty/spdlog/include
    ${PROJECT_SOURCE_DIR}/ThirdParty/mujoco
    ${PROJECT_SOURCE_DIR}/ThirdParty/glfw
    ${PROJECT_SOURCE_DIR}/ThirdParty/gl/include
    ${PROJECT_SOURCE_DIR}/DataPackage/include
    ${PROJECT_SOURCE_DIR}/StateMachine/include
    ${PROJECT_SOURCE_DIR}/LowPassFilter/include
    ${PROJECT_SOURCE_DIR}/ZeroState/include
    ${PROJECT_SOURCE_DIR}/RL_deploy_cpg/include
    ${PHYBOT_JOYSTICK_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/MotorList/include
    ${PROJECT_SOURCE_DIR}/MujocoInterface/include
    ${PROJECT_SOURCE_DIR}/MujocoInterface/include/simulate
    ${PROJECT_SOURCE_DIR}/device/IMU
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/fw_downloader
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/lib
)

# Both joystick implementations expose a same-named header and class with
# different layouts. Never add the unselected implementation to the include
# path, otherwise main and the prebuilt joystick library have an ABI mismatch.
function(phybot_collect_include_dirs directory)
    file(GLOB children LIST_DIRECTORIES true "${directory}/*")
    foreach(child IN LISTS children)
        if(NOT IS_DIRECTORY "${child}" OR IS_SYMLINK "${child}")
            continue()
        endif()
        if(child STREQUAL "${PROJECT_SOURCE_DIR}/ThirdParty" OR
           child STREQUAL "${PROJECT_SOURCE_DIR}/Joystick" OR
           child STREQUAL "${PROJECT_SOURCE_DIR}/Model_airplane_Joystick")
            continue()
        endif()
        get_filename_component(name "${child}" NAME)
        if(name MATCHES "^(build.*|prebuilt|lib|config|model|data|resources|assets)$")
            continue()
        elseif(name STREQUAL "include")
            include_directories("${child}")
        else()
            phybot_collect_include_dirs("${child}")
        endif()
    endforeach()
endfunction()
phybot_collect_include_dirs("${PROJECT_SOURCE_DIR}")

link_directories(
    ${PROJECT_SOURCE_DIR}/lib
    ${PROJECT_SOURCE_DIR}/MotorList/lib
    ${PROJECT_SOURCE_DIR}/ThirdParty/boost/lib
    ${PROJECT_SOURCE_DIR}/ThirdParty/yaml/lib
    ${PROJECT_SOURCE_DIR}/ThirdParty/mujoco/lib
    ${PROJECT_SOURCE_DIR}/ThirdParty/glfw/lib
)

if(NOT DEFINED WHICH_ENV)
    set(WHICH_ENV "realrobot_mini" CACHE STRING "Build environment" FORCE)
endif()

set(PREBUILT_DIR "${PROJECT_SOURCE_DIR}/prebuilt/${IMU_TYPE}/${WHICH_ENV}")
if(NOT EXISTS "${PREBUILT_DIR}/libphybot_common.a")
    message(FATAL_ERROR "Missing prebuilt libraries for ${IMU_TYPE}/${WHICH_ENV}: ${PREBUILT_DIR}")
endif()

set(PHYBOT_COMMON_LIBS
    "${PREBUILT_DIR}/libphybot_common.a"
    "${PREBUILT_DIR}/libphybot_joystick.a"
    ${TORCH_LIBRARIES}
    pthread
    yaml-cpp
)

if(WHICH_ENV STREQUAL "realrobot_mini")
    if(NOT EXISTS "${PREBUILT_DIR}/libphybot_realrobot.a")
        message(FATAL_ERROR "Missing ${PREBUILT_DIR}/libphybot_realrobot.a")
    endif()
    add_executable(main ${PROJECT_SOURCE_DIR}/RobotStart/realrobot/run_mini.cpp)
    target_link_libraries(main PUBLIC
        "${PREBUILT_DIR}/libphybot_realrobot.a"
        ${PHYBOT_COMMON_LIBS}
        MotorDrive
    )
elseif(WHICH_ENV STREQUAL "test")
    if(NOT EXISTS "${PREBUILT_DIR}/libphybot_test.a")
        message(FATAL_ERROR "Missing ${PREBUILT_DIR}/libphybot_test.a")
    endif()
    set(PHYBOT_TEST_LIBS "${PREBUILT_DIR}/libphybot_test.a" ${PHYBOT_COMMON_LIBS} MotorDrive)
    add_executable(set_zero ${PROJECT_SOURCE_DIR}/RobotStart/realrobot/set_zero_mini.cpp)
    target_link_libraries(set_zero PUBLIC ${PHYBOT_TEST_LIBS})
    add_executable(set_one_zero ${PROJECT_SOURCE_DIR}/RobotStart/test/set_one_zero.cpp)
    target_link_libraries(set_one_zero PUBLIC ${PHYBOT_TEST_LIBS})
    add_executable(head_test ${PROJECT_SOURCE_DIR}/RobotStart/test/head_test.cpp)
    target_link_libraries(head_test PUBLIC ${PHYBOT_TEST_LIBS})
    add_executable(joy_test ${PROJECT_SOURCE_DIR}/RobotStart/test/joy_test.cpp)
    target_link_libraries(joy_test PUBLIC ${PHYBOT_TEST_LIBS})
    add_executable(read_pos ${PROJECT_SOURCE_DIR}/RobotStart/test/read_pos.cpp)
    target_link_libraries(read_pos PUBLIC ${PHYBOT_TEST_LIBS})
    add_executable(set_one_Enable ${PROJECT_SOURCE_DIR}/RobotStart/test/set_one_Enable.cpp)
    target_link_libraries(set_one_Enable PUBLIC ${PHYBOT_TEST_LIBS})
    if(EXISTS ${PROJECT_SOURCE_DIR}/RobotStart/test/imu_test.cpp)
        add_executable(imu_test ${PROJECT_SOURCE_DIR}/RobotStart/test/imu_test.cpp)
        target_link_libraries(imu_test PUBLIC ${PHYBOT_TEST_LIBS})
    endif()
elseif(WHICH_ENV STREQUAL "mujoco_sim_mini")
    if(NOT EXISTS "${PREBUILT_DIR}/libphybot_mujoco.a")
        message(FATAL_ERROR "Missing ${PREBUILT_DIR}/libphybot_mujoco.a")
    endif()
    add_executable(main ${PROJECT_SOURCE_DIR}/RobotStart/mujoco_sim/main.cc)
    target_link_libraries(main PUBLIC
        "${PREBUILT_DIR}/libphybot_mujoco.a"
        ${PHYBOT_COMMON_LIBS}
        mujoco
        glfw3
    )
else()
    message(FATAL_ERROR "Unsupported environment: ${WHICH_ENV}")
endif()
