# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
include_directories($ENV{IDF_PATH}/components/esp_gatts/include)
include_directories($ENV{IDF_PATH}/components/esp_gatt_common/include)
include_directories($ENV{IDF_PATH}/components/esp_gap/include)
include_directories("C:/Users/krzys/esp/v5.3.1/esp-idf/components/bt/host/bluedroid/api/include/api")
include_directories("C:/Users/krzys/esp/v5.3.1/esp-idf/components/bt/include/esp32/include")


project(final_project)
