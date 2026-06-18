cmake_minimum_required(VERSION 3.12)

project(phybot_customer_dev LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(PHYBOT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
set(PHYBOT_LIB_DIR "${PHYBOT_ROOT}/lib")
set(PHYBOT_PREBUILT_LIB_DIR "${PHYBOT_ROOT}/prebuilt/lib")
set(PHYBOT_OBJECT_DIR "${PHYBOT_ROOT}/prebuilt/objects")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PHYBOT_ROOT}/bin")

find_package(Threads REQUIRED)

add_compile_options(-Wno-return-type)
add_compile_definitions(
    _GLIBCXX_USE_CXX11_ABI=1
    USE_C10D_GLOO
    USE_DISTRIBUTED
    USE_RPC
    USE_TENSORPIPE
)

set(PHYBOT_INCLUDE_DIRS
    "${PHYBOT_ROOT}"
    "${PHYBOT_ROOT}/DataPackage/include"
    "${PHYBOT_ROOT}/StateMachine/include"
    "${PHYBOT_ROOT}/LowPassFilter/include"
    "${PHYBOT_ROOT}/ZeroState/include"
    "${PHYBOT_ROOT}/RL_deploy_cpg/include"
    "${PHYBOT_ROOT}/MotorList/include"
    "${PHYBOT_ROOT}/Model_airplane_Joystick/include"
    "${PHYBOT_ROOT}/MujocoInterface/include"
    "${PHYBOT_ROOT}/device/Imu_hipnuc/linux"
    "${PHYBOT_ROOT}/device/Imu_hipnuc/linux/fw_downloader"
    "${PHYBOT_ROOT}/device/Imu_hipnuc/lib"
    "${PHYBOT_ROOT}/ThirdParty"
    "${PHYBOT_ROOT}/ThirdParty/eigen3"
    "${PHYBOT_ROOT}/ThirdParty/yaml/include"
    "${PHYBOT_ROOT}/ThirdParty/libtorch/include"
    "${PHYBOT_ROOT}/ThirdParty/libtorch/include/torch/csrc/api/include"
    "${PHYBOT_ROOT}/ThirdParty/mujoco"
    "${PHYBOT_ROOT}/ThirdParty/glfw"
    "${PHYBOT_ROOT}/ThirdParty/gl/include"
)

set(PHYBOT_OPEN_SOURCES
    "${PHYBOT_ROOT}/device/Imu_hipnuc/linux/HipnucReader.cpp"
    "${PHYBOT_ROOT}/device/Imu_hipnuc/linux/serial_port.c"
    "${PHYBOT_ROOT}/device/Imu_hipnuc/linux/log.c"
    "${PHYBOT_ROOT}/device/Imu_hipnuc/linux/fw_downloader/hex2bin.c"
    "${PHYBOT_ROOT}/device/Imu_hipnuc/linux/fw_downloader/kboot.c"
    "${PHYBOT_ROOT}/device/Imu_hipnuc/lib/example_data.c"
    "${PHYBOT_ROOT}/device/Imu_hipnuc/lib/hipnuc_dec.c"
    "${PHYBOT_ROOT}/device/Imu_hipnuc/lib/nmea_dec.c"
    "${PHYBOT_ROOT}/StateMachine/src/statemachinemanager.cpp"
    "${PHYBOT_ROOT}/ZeroState/src/ZeroState.cpp"
    "${PHYBOT_ROOT}/RL_deploy_cpg/src/rl_deploy.cpp"
)

add_library(phybot_open STATIC ${PHYBOT_OPEN_SOURCES})
target_include_directories(phybot_open PUBLIC ${PHYBOT_INCLUDE_DIRS})

set(PHYBOT_EXTERNAL_LIBS
    "${PHYBOT_LIB_DIR}/libtorch.so"
    "-Wl,--no-as-needed"
    "${PHYBOT_LIB_DIR}/libtorch_cpu.so"
    "-Wl,--as-needed"
    "${PHYBOT_LIB_DIR}/libc10.so"
    "${PHYBOT_LIB_DIR}/libkineto.a"
    Threads::Threads
    "${PHYBOT_LIB_DIR}/libyaml-cpp.so"
)

set(PHYBOT_COMMON_LINK_LIBS
    phybot_open
    "${PHYBOT_PREBUILT_LIB_DIR}/libphybot_joystick_closed.a"
    "${PHYBOT_PREBUILT_LIB_DIR}/libphybot_common_closed.a"
    ${PHYBOT_EXTERNAL_LIBS}
)

function(configure_phybot_target target)
    target_include_directories("${target}" PRIVATE ${PHYBOT_INCLUDE_DIRS})
    set_target_properties("${target}" PROPERTIES
        LINKER_LANGUAGE CXX
        BUILD_RPATH "$ORIGIN/../lib"
        INSTALL_RPATH "$ORIGIN/../lib"
        RUNTIME_OUTPUT_DIRECTORY "${PHYBOT_ROOT}/bin"
    )
endfunction()

function(add_prebuilt_executable target object_file)
    add_executable("${target}" "${object_file}")
    configure_phybot_target("${target}")
endfunction()

add_prebuilt_executable(realrobot_mini "${PHYBOT_OBJECT_DIR}/realrobot_mini.o")
target_link_libraries(realrobot_mini PRIVATE
    "${PHYBOT_PREBUILT_LIB_DIR}/libphybot_realrobot.a"
    "${PHYBOT_LIB_DIR}/libMotorDrive.so"
    ${PHYBOT_COMMON_LINK_LIBS}
)

add_prebuilt_executable(mujoco_sim_mini "${PHYBOT_OBJECT_DIR}/mujoco_sim_mini.o")
target_link_libraries(mujoco_sim_mini PRIVATE
    "${PHYBOT_PREBUILT_LIB_DIR}/libphybot_mujoco.a"
    "${PHYBOT_LIB_DIR}/libmujoco.so"
    "${PHYBOT_LIB_DIR}/libglfw3.so"
    ${PHYBOT_COMMON_LINK_LIBS}
)

set(PHYBOT_TEST_LINK_LIBS
    "${PHYBOT_PREBUILT_LIB_DIR}/libphybot_test.a"
    "${PHYBOT_LIB_DIR}/libMotorDrive.so"
    ${PHYBOT_COMMON_LINK_LIBS}
)

set(PHYBOT_PREBUILT_TEST_TARGETS
    set_zero
    set_one_zero
    head_test
    read_pos
    set_one_Enable
)

foreach(target_name IN LISTS PHYBOT_PREBUILT_TEST_TARGETS)
    add_prebuilt_executable("${target_name}" "${PHYBOT_OBJECT_DIR}/${target_name}.o")
    target_link_libraries("${target_name}" PRIVATE ${PHYBOT_TEST_LINK_LIBS})
endforeach()

add_executable(imu_test "${PHYBOT_ROOT}/RobotStart/test/imu_test.cpp")
configure_phybot_target(imu_test)
target_link_libraries(imu_test PRIVATE ${PHYBOT_TEST_LINK_LIBS})

add_executable(joy_test "${PHYBOT_ROOT}/RobotStart/test/joy_test.cpp")
configure_phybot_target(joy_test)
target_link_libraries(joy_test PRIVATE ${PHYBOT_TEST_LINK_LIBS})

add_executable(motor_control "${PHYBOT_ROOT}/RobotStart/test/motor_control.cpp")
configure_phybot_target(motor_control)
target_link_libraries(motor_control PRIVATE ${PHYBOT_TEST_LINK_LIBS})

add_executable(pd_control "${PHYBOT_ROOT}/RobotStart/test/pd_control.cpp")
configure_phybot_target(pd_control)
target_link_libraries(pd_control PRIVATE ${PHYBOT_TEST_LINK_LIBS})
