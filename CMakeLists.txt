#[[
This CMakeLists.txt is designed to integrate with CLion (great IDE, free for personal use!) and use arduino-cli for building and uploading an ESP32 project.
It extracts the necessary include paths from arduino-cli's output to ensure that CLion can properly index the project files, providing features like code completion and error highlighting.
If you want to use this CMakeLists.txt, make sure to adjust the configuration section with the correct settings for your environment.
]]
cmake_minimum_required(VERSION 3.20)
project(ESP32_Pinball_Controller_Composite)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

##############################################################################
# Configuration
##############################################################################
set(ARDUINO_CLI "arduino-cli")              # Path to arduino-cli
set(FQBN "esp32:esp32:esp32s3")             # Fully Qualified Board Name
set(PORT "COM8")                            # Serial port for uploading (e.g., COM3 on Windows or /dev/ttyUSB0 on Linux)
set(BOARD_OPTIONS "UploadSpeed=921600")     # Board options for the ESP32 upload
set(USER_LIBS_DIRS                          # User libraries to include
        "ESP32-BLE-CompositeHID"
        "MPU6050"
)
#set(ARDUINO_EXTRA_FLAGS                     # Extra flags for compilation
#        "-DGAMEPAD_ENABLE=true"
#)

##############################################################################
# Get compilation properties via arduino-cli
##############################################################################
execute_process(
        COMMAND ${ARDUINO_CLI} compile --fqbn ${FQBN} --show-properties
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE ARDUINO_PROPERTIES
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
)

# Helper function to extract a property from the raw output
function(extract_arduino_property PROP_NAME OUTPUT_VAR INPUT_TEXT)
    string(REGEX MATCH "${PROP_NAME}=([^\n]+)" MATCHED "${INPUT_TEXT}")
    if (MATCHED)
        string(REGEX REPLACE "${PROP_NAME}=" "" RESULT "${MATCHED}")
        # Remove any carriage returns (Windows)
        string(STRIP "${RESULT}" RESULT)
        set(${OUTPUT_VAR} "${RESULT}" PARENT_SCOPE)
    endif ()
endfunction()

# Extract include paths
extract_arduino_property("build.core.path" CORE_PATH "${ARDUINO_PROPERTIES}")
extract_arduino_property("build.variant.path" VARIANT_PATH "${ARDUINO_PROPERTIES}")
extract_arduino_property("runtime.platform.path" PLATFORM_PATH "${ARDUINO_PROPERTIES}")
extract_arduino_property("compiler.sdk.path" SDK_PATH "${ARDUINO_PROPERTIES}")


##############################################################################
# Get the user libraries folder
##############################################################################
execute_process(
        COMMAND arduino-cli config get directories.user
        OUTPUT_VARIABLE ARDUINO_USER_DIR
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(ARDUINO_LIBRARIES_DIR "${ARDUINO_USER_DIR}/libraries")


##############################################################################
# Configure the target for the IDE
##############################################################################
file(GLOB_RECURSE SOURCES "*.ino" "*.cpp" "*.c")

# Force the language (CLion does not know that .ino is C++)
set_source_files_properties(${SOURCES} PROPERTIES LANGUAGE CXX)

# Create a "dummy" executable so that CLion indexes the files
add_executable(${PROJECT_NAME} ${SOURCES})


##############################################################################
# Injection of Includes
##############################################################################
if (CORE_PATH)
    target_include_directories(${PROJECT_NAME} PRIVATE ${CORE_PATH})
    message(STATUS "Core path: ${CORE_PATH}")
endif ()

if (VARIANT_PATH)
    target_include_directories(${PROJECT_NAME} PRIVATE ${VARIANT_PATH})
    message(STATUS "Variant path: ${VARIANT_PATH}")
endif ()

# Basic platform includes (Arduino.h, etc.)
if (PLATFORM_PATH)
    target_include_directories(${PROJECT_NAME} PRIVATE
            ${PLATFORM_PATH}/tools/sdk/include
            ${PLATFORM_PATH}/tools/sdk/esp32/include
    )

    message(STATUS "Platform path: ${PLATFORM_PATH}")
endif ()

# SDK ESP32 includes (FreeRTOS, Drivers, etc.)
if (SDK_PATH)
    message(STATUS "SDK path: ${SDK_PATH}")

    file(GLOB SDK_INCLUDE_DIRS "${SDK_PATH}/include/*/include")

    foreach (SDK_DIR ${SDK_INCLUDE_DIRS})
        if (IS_DIRECTORY ${SDK_DIR})
            target_include_directories(${PROJECT_NAME} PRIVATE ${SDK_DIR})
        endif ()
    endforeach ()
endif ()


# User libraries includes
foreach (USER_LIB_DIR ${USER_LIBS_DIRS})
    if (IS_DIRECTORY "${ARDUINO_LIBRARIES_DIR}/${USER_LIB_DIR}/src")
        target_include_directories(${PROJECT_NAME} PRIVATE "${ARDUINO_LIBRARIES_DIR}/${USER_LIB_DIR}/src")
        message(STATUS "${USER_LIB_DIR} path: ${ARDUINO_LIBRARIES_DIR}/${USER_LIB_DIR}/src")
    elseif (IS_DIRECTORY "${ARDUINO_LIBRARIES_DIR}/${USER_LIB_DIR}")
        target_include_directories(${PROJECT_NAME} PRIVATE "${ARDUINO_LIBRARIES_DIR}/${USER_LIB_DIR}")
        message(STATUS "${USER_LIB_DIR} path: ${ARDUINO_LIBRARIES_DIR}/${USER_LIB_DIR}")
    else ()
        message(WARNING "Library path not found: ${ARDUINO_LIBRARIES_DIR}/${USER_LIB_DIR}")
    endif ()
endforeach ()

# Indexing and syntax highlighting
target_compile_definitions(${PROJECT_NAME} PUBLIC
        CONFIG_BT_ENABLED=1
        GAMEPAD_ENABLE=true
)

##############################################################################
# Custom Commands (Build / Upload)
##############################################################################

add_custom_target(arduino_compile
        COMMAND ${ARDUINO_CLI} compile -v --fqbn ${FQBN} --build-property "compiler.cpp.extra_flags=\"${ARDUINO_EXTRA_FLAGS}\""
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT " Compiling with arduino-cli..."
)

add_custom_target(arduino_compile_upload
        COMMAND ${ARDUINO_CLI} compile -v -u -l serial -p ${PORT} --board-options ${BOARD_OPTIONS} --fqbn ${FQBN} --build-property "compiler.cpp.extra_flags=\"${ARDUINO_EXTRA_FLAGS}\""
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Compiling and uploading with arduino-cli..."
)

add_custom_target(arduino_upload
        COMMAND ${ARDUINO_CLI} upload -v --port ${PORT} --board-options ${BOARD_OPTIONS} --fqbn ${FQBN}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Uploading to board..."
        DEPENDS arduino_compile
)

add_custom_target(arduino_monitor
        COMMAND ${ARDUINO_CLI} monitor --port ${PORT}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Opening serial monitor..."
)

add_custom_target(arduino_clean
        COMMAND ${ARDUINO_CLI} cache clean
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Cleaning arduino-cli cache..."
)



