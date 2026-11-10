cmake_minimum_required(VERSION 3.20)
project(lumo_p2_locomotion_deploy VERSION 1.0.0 LANGUAGES CXX)

include(CTest)
find_package(Threads REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(LUMO_BUILD_SIM "Build the MuJoCo sim2sim executable" ON)
option(LUMO_BUILD_REAL "Build the Dobot Lumo P2 real-robot executable" OFF)
set(LUMO_DEPS_ROOT "" CACHE PATH "Dependency prefix containing include/ and lib/")
set(LUMO_SDK_ROOT "" CACHE PATH "Path to the public dobot-sdk repository")

if(NOT LUMO_DEPS_ROOT AND DEFINED ENV{LUMO_DEPS_ROOT})
    set(LUMO_DEPS_ROOT "$ENV{LUMO_DEPS_ROOT}" CACHE PATH "Dependency prefix" FORCE)
endif()
if(NOT LUMO_SDK_ROOT AND DEFINED ENV{LUMO_SDK_ROOT})
    set(LUMO_SDK_ROOT "$ENV{LUMO_SDK_ROOT}" CACHE PATH "Public dobot-sdk" FORCE)
endif()
if(NOT EXISTS "${LUMO_DEPS_ROOT}/include" OR NOT EXISTS "${LUMO_DEPS_ROOT}/lib")
    message(FATAL_ERROR
        "LUMO_DEPS_ROOT must point to a dependency prefix with include/ and lib/.")
endif()

function(lumo_first_library output)
    foreach(candidate IN LISTS ARGN)
        if(EXISTS "${candidate}")
            set(${output} "${candidate}" PARENT_SCOPE)
            return()
        endif()
    endforeach()
    set(${output} "" PARENT_SCOPE)
endfunction()

lumo_first_library(ONNXRUNTIME_LIBRARY
    "${LUMO_DEPS_ROOT}/lib/libonnxruntime.so"
    "${LUMO_DEPS_ROOT}/lib/libonnxruntime.dylib")
if(NOT ONNXRUNTIME_LIBRARY)
    message(FATAL_ERROR "ONNX Runtime was not found under LUMO_DEPS_ROOT/lib")
endif()

add_library(onnxruntime SHARED IMPORTED GLOBAL)
set_target_properties(onnxruntime PROPERTIES
    IMPORTED_LOCATION "${ONNXRUNTIME_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${LUMO_DEPS_ROOT}/include")

add_library(lumo_deploy_core STATIC
    src/config.cpp
    src/controller.cpp
    src/policy.cpp)
target_include_directories(lumo_deploy_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories(lumo_deploy_core SYSTEM PUBLIC
    "${LUMO_DEPS_ROOT}/include/eigen3")
target_link_libraries(lumo_deploy_core PUBLIC onnxruntime Threads::Threads)
target_compile_options(lumo_deploy_core PRIVATE
    -Wall -Wextra -Wpedantic -Werror
    $<$<CXX_COMPILER_ID:GNU>:-Wno-error=class-memaccess>)

if(LUMO_BUILD_SIM)
    find_package(OpenGL REQUIRED)
    lumo_first_library(MUJOCO_LIBRARY
        "${LUMO_DEPS_ROOT}/lib/libmujoco.so"
        "${LUMO_DEPS_ROOT}/lib/libmujoco.dylib")
    lumo_first_library(GLFW_LIBRARY
        "${LUMO_DEPS_ROOT}/lib/libglfw.so"
        "${LUMO_DEPS_ROOT}/lib/libglfw.dylib"
        "${LUMO_DEPS_ROOT}/lib/libglfw.a")
    if(NOT MUJOCO_LIBRARY OR NOT GLFW_LIBRARY)
        message(FATAL_ERROR "MuJoCo and GLFW are required for LUMO_BUILD_SIM")
    endif()

    add_library(mujoco SHARED IMPORTED GLOBAL)
    set_target_properties(mujoco PROPERTIES
        IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${LUMO_DEPS_ROOT}/include")
    add_library(glfw UNKNOWN IMPORTED GLOBAL)
    set_target_properties(glfw PROPERTIES
        IMPORTED_LOCATION "${GLFW_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${LUMO_DEPS_ROOT}/include")

    add_executable(lumo_sim src/sim_main.cpp)
    target_compile_definitions(lumo_sim PRIVATE
        LUMO_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(lumo_sim PRIVATE
        lumo_deploy_core mujoco glfw OpenGL::GL ${CMAKE_DL_LIBS})
    target_compile_options(lumo_sim PRIVATE -Wall -Wextra -Wpedantic -Werror)
    set_target_properties(lumo_sim PROPERTIES BUILD_RPATH "${LUMO_DEPS_ROOT}/lib")
endif()

if(LUMO_BUILD_REAL)
    string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" LUMO_TARGET_PROCESSOR)
    if(NOT LUMO_TARGET_PROCESSOR MATCHES "^(x86_64|amd64)$")
        message(FATAL_ERROR
            "LUMO_BUILD_REAL supports only an x86_64 Linux development computer.")
    endif()
    if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
        message(FATAL_ERROR "LUMO_BUILD_REAL requires Linux.")
    endif()
    foreach(required IN ITEMS
            "CMakeLists.txt"
            "include/dobot/sdk/session.hpp"
            "lib/x86_64/libdobot_sdk.a"
            "lib/x86_64/libdobot_log.so"
            "thirdparty/lib/x86_64/libfastdds.so"
            "thirdparty/lib/x86_64/libfastcdr.so")
        if(NOT EXISTS "${LUMO_SDK_ROOT}/${required}")
            message(FATAL_ERROR
                "LUMO_SDK_ROOT must point to the public dobot-sdk repository; missing ${required}")
        endif()
    endforeach()

    # Consume the public SDK target, including its matching DDS/logging libraries.
    set(BUILD_EXAMPLES OFF)
    set(TARGET_ARCH x86_64)
    add_subdirectory("${LUMO_SDK_ROOT}" "${CMAKE_BINARY_DIR}/dobot-sdk" EXCLUDE_FROM_ALL)

    add_library(lumo_sdk_adapter STATIC src/sdk_adapter.cpp)
    target_link_libraries(lumo_sdk_adapter PUBLIC lumo_deploy_core dobot_sdk)
    target_compile_options(lumo_sdk_adapter PRIVATE -Wall -Wextra -Wpedantic -Werror)

    add_library(lumo_csv_recorder STATIC src/csv_recorder.cpp)
    target_link_libraries(lumo_csv_recorder PUBLIC lumo_deploy_core)
    target_compile_options(lumo_csv_recorder PRIVATE -Wall -Wextra -Wpedantic -Werror)

    add_executable(lumo_real src/real_main.cpp)
    target_compile_definitions(lumo_real PRIVATE
        LUMO_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(lumo_real PRIVATE lumo_sdk_adapter lumo_csv_recorder)
    target_compile_options(lumo_real PRIVATE -Wall -Wextra -Wpedantic -Werror)
    set_target_properties(lumo_real PROPERTIES BUILD_RPATH
        "${LUMO_DEPS_ROOT}/lib;${LUMO_SDK_ROOT}/lib/x86_64;${LUMO_SDK_ROOT}/thirdparty/lib/x86_64")

    if(BUILD_TESTING)
        add_executable(sdk_adapter_test tests/sdk_adapter_test.cpp)
        target_link_libraries(sdk_adapter_test PRIVATE lumo_sdk_adapter)
        target_compile_options(sdk_adapter_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
        add_test(NAME sdk_adapter_test COMMAND sdk_adapter_test)
        add_test(NAME real_asset_check COMMAND lumo_real --check)
        add_executable(runtime_alert_test tests/runtime_alert_test.cpp)
        target_include_directories(runtime_alert_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
        target_compile_options(runtime_alert_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
        add_test(NAME runtime_alert_test COMMAND runtime_alert_test)
        add_executable(csv_recorder_test tests/csv_recorder_test.cpp)
        target_link_libraries(csv_recorder_test PRIVATE lumo_csv_recorder)
        target_compile_options(csv_recorder_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
        add_test(NAME csv_recorder_test COMMAND csv_recorder_test)
    endif()
endif()

if(BUILD_TESTING)
    add_executable(fsm_test tests/fsm_test.cpp)
    target_link_libraries(fsm_test PRIVATE lumo_deploy_core)
    target_compile_options(fsm_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
    add_test(NAME fsm_test COMMAND fsm_test)

    add_executable(policy_inference_test tests/policy_inference_test.cpp)
    target_compile_definitions(policy_inference_test PRIVATE
        LUMO_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(policy_inference_test PRIVATE lumo_deploy_core)
    target_compile_options(policy_inference_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
    add_test(NAME policy_inference_test COMMAND policy_inference_test)
    if(LUMO_BUILD_SIM)
        add_test(NAME asset_check COMMAND lumo_sim --check)
    endif()
endif()
