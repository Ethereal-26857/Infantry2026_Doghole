# ##############################################################################
# #################        CMake Template (CUSTOM)       ######################
# #################    Copyright (c) 2023 Hello World    ######################
# ##############################################################################

# Set the system name and version
set(CMAKE_SYSTEM_VERSION 1)

# Specify the minimum required version of CMake
cmake_minimum_required(VERSION 3.22)

include("cmake/gcc-arm-none-eabi.cmake")

add_definitions(-DUSE_CUSTOM_DEFINE_ASSERT=1)
# Set the C++ and C standards
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Set the library path
set(CMAKE_LIBRARY_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)

# ########################## USER CONFIG SECTION ##############################
# Set the project name and the languages used
project(XiaoNeZha C CXX ASM) # TODO: change project name here

# Specify user folders
set(user_folders "Middlewares" "USB_DEVICE" "SentryComponents" "SentryModules" "Instance" "Task") # TODO Add your own user folders here

# Specify the path to the HW-Components directory
set(HWC_DIR "HW-Components") # TODO: Set your own HW-Components path here

# Include utility functions and default configuration
include("${HWC_DIR}/cmake/utils/function_tools.cmake")
include("${HWC_DIR}/config.cmake") # Default configuration

# TODO: Overwrite default configuration instead of changing it in file
# tools
set(use_hwcomponents_tools ON)
set(use_prebuilt_hwcomponents_tools ON)
set(use_hwcomponents_custom_define_assert ON)

# algorithms
set(use_hwcomponents_algorithms_controller ON)
set(use_prebuilt_hwcomponents_algorithms_controller ON)

set(use_hwcomponents_algorithms_ik_solver_chassis ON)
set(use_prebuilt_hwcomponents_algorithms_ik_solver_chassis ON)
set(CHASSIS_IKSOLVER_USE_ARM_MATH ON)

set(use_hwcomponents_algorithms_ahrs ON)
set(use_prebuilt_hwcomponents_algorithms_ahrs ON)

set(use_hwcomponents_algorithms_filter ON)
set(use_prebuilt_hwcomponents_algorithms_filter ON)

set(use_hwcomponents_algorithms_observer ON)
set(use_prebuilt_hwcomponents_algorithms_observer ON)

set(use_hwcomponents_algorithms_fsm ON)
set(use_prebuilt_hwcomponents_algorithms_fsm ON)

# devices
set(use_hwcomponents_devices_referee ON)
set(use_prebuilt_hwcomponents_devices_referee OFF)

set(use_hwcomponents_devices_motor ON)
set(use_prebuilt_hwcomponents_devices_motor OFF)

set(use_hwcomponents_devices_laser ON)
set(use_prebuilt_hwcomponents_devices_laser ON)

set(use_hwcomponents_devices_servo OFF)
set(use_prebuilt_hwcomponents_devices_servo OFF)

set(use_hwcomponents_devices_buzzer ON)
set(use_prebuilt_hwcomponents_devices_buzzer ON)

set(use_hwcomponents_devices_remote_control ON)
set(use_prebuilt_hwcomponents_devices_remote_control ON)

set(use_hwcomponents_devices_imu ON)
set(use_prebuilt_hwcomponents_devices_imu ON)

set(use_hwcomponents_devices_vision ON)
set(use_prebuilt_hwcomponents_devices_vision ON)

# dsp
set(use_hwcomponents_bsp_tick ON)
set(use_prebuilt_hwcomponents_bsp_tick ON)

# Third_Party
set(use_hwcomponents_third_party_dsp ON)
set(use_prebuilt_hwcomponents_third_party_dsp ON)

set(use_prebuilt_hwcomponents_modules OFF)

# TODO: Add your own `config.cmake` file or set your own configuration here

# Enable preprocessing for assembler files
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

# Disable some warnings
set(COM_FLAGS "-Wno-unused-parameter -Wno-missing-field-initializers -Wno-pedantic -Wno-unknown-pragmas")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COM_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COM_FLAGS} -Wno-reorder")

# add_subdirectory(cmake/stm32cubemx)
# get_target_property(STM32_COMPILE_DEFINES stm32cubemx INTERFACE_COMPILE_DEFINITIONS)
# list(APPEND STM32_COMPILE_DEFINES DEBUG DEBUG)

# foreach(STM32_COMPILE_DEFINE ${STM32_COMPILE_DEFINES})
#   # Exclude the element if it starts with "$"
#   if(STM32_COMPILE_DEFINE MATCHES "^\\$")
#     continue()
#   endif()

#   add_definitions(-D${STM32_COMPILE_DEFINE})

#   # Get HAL filename
#   if(STM32_COMPILE_DEFINE MATCHES "STM32[A-Z][0-9]")
#     string(SUBSTRING ${STM32_COMPILE_DEFINE} 0 7 STM32_DEVICE)
#     string(TOLOWER ${STM32_DEVICE} stm32_hal_filename)
#     string(CONCAT stm32_hal_filename ${stm32_hal_filename} "xx_hal.h")
#     message(STATUS "HAL file name: ${stm32_hal_filename}")

#     # Set macro for the HAL filename
#     add_definitions(-DSTM32_HAL_FILENAME="${stm32_hal_filename}")
#   endif()
# endforeach()

# #################### ADD LIBRARIES AND EXECUTABLE SECTION ####################
# Initialize source and include lists
set(project_srcs)
set(project_incs)

# Search for include files and source files in the Core directory
search_incs_recurse("${CMAKE_CURRENT_SOURCE_DIR}/Core" core_incs)
file(GLOB_RECURSE core_srcs "Core/*.*")
list(APPEND project_srcs ${core_srcs})
list(APPEND project_incs ${core_incs})

# Search for include files and source files in the Drivers directory
search_incs_recurse("${CMAKE_CURRENT_SOURCE_DIR}/Drivers" drivers_incs)
file(GLOB_RECURSE drivers_srcs "Drivers/*.*")
list(APPEND project_incs ${drivers_incs})

# For each user folder, search for include files and source files
foreach(user_folder ${user_folders})
  search_incs_recurse("${CMAKE_CURRENT_SOURCE_DIR}/${user_folder}"
    ${user_folder}_incs)
  file(GLOB_RECURSE ${user_folder}_srcs "${user_folder}/*.*")
  list(APPEND project_incs ${${user_folder}_incs})
  list(APPEND project_srcs ${${user_folder}_srcs})
endforeach()

# Add a static library for the drivers
add_library(drivers STATIC ${drivers_srcs})
target_include_directories(drivers PUBLIC ${core_incs} ${drivers_incs})

# Add the HW-Components directory as a subdirectory
# add_subdirectory(${HWC_DIR})

# Add an executable for the project
file(GLOB_RECURSE startup_file "*.s")
add_executable(${PROJECT_NAME} ${project_srcs} ${startup_file})
message(STATUS "Project name: ${PROJECT_NAME}")

# 如果要更新cube 则把这个部分代码丢下来
add_subdirectory(cmake/stm32cubemx)
get_target_property(STM32_COMPILE_DEFINES stm32cubemx INTERFACE_COMPILE_DEFINITIONS)
list(APPEND STM32_COMPILE_DEFINES DEBUG DEBUG)

foreach(STM32_COMPILE_DEFINE ${STM32_COMPILE_DEFINES})
  # Exclude the element if it starts with "$"
  if(STM32_COMPILE_DEFINE MATCHES "^\\$")
    continue()
  endif()

  add_definitions(-D${STM32_COMPILE_DEFINE})

  # Get HAL filename
  if(STM32_COMPILE_DEFINE MATCHES "STM32[A-Z][0-9]")
    string(SUBSTRING ${STM32_COMPILE_DEFINE} 0 7 STM32_DEVICE)
    string(TOLOWER ${STM32_DEVICE} stm32_hal_filename)
    string(CONCAT stm32_hal_filename ${stm32_hal_filename} "xx_hal.h")
    message(STATUS "HAL file name: ${stm32_hal_filename}")

    # Set macro for the HAL filename
    add_definitions(-DSTM32_HAL_FILENAME="${stm32_hal_filename}")
  endif()
endforeach()

#还要把这个丢下来
add_subdirectory(${HWC_DIR})

# Add the project includes to the executable
target_include_directories(${PROJECT_NAME} PUBLIC ${project_incs})
target_include_directories(${PROJECT_NAME} PUBLIC ${${HWC_LIB_PREFIX}_incs})

# Link the drivers library and the HW-Components library to the executable
# 去除public
target_link_libraries(${PROJECT_NAME} PUBLIC drivers)
target_link_libraries(${PROJECT_NAME} PUBLIC ${${HWC_LIB_PREFIX}_libs})

# Define the output hex and bin files
set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

# Add a post-build command to generate the hex and bin files
add_custom_command(
  TARGET ${PROJECT_NAME}
  POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}> ${HEX_FILE}
  COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}>
  ${BIN_FILE}
  COMMENT "Building ${HEX_FILE} Building ${BIN_FILE}")
