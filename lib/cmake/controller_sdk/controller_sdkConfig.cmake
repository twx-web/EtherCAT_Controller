# Relocatable binary SDK package
get_filename_component(_CONTROLLER_SDK_CMAKE_DIR "${CMAKE_CURRENT_LIST_DIR}" ABSOLUTE)
get_filename_component(_CONTROLLER_SDK_ROOT "${_CONTROLLER_SDK_CMAKE_DIR}/../../.." ABSOLUTE)

if(NOT TARGET controller_sdk::controller_sdk)
  add_library(controller_sdk::controller_sdk SHARED IMPORTED)
  set_target_properties(controller_sdk::controller_sdk PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${_CONTROLLER_SDK_ROOT}/include/controller_sdk;${_CONTROLLER_SDK_ROOT}/include"
    IMPORTED_LOCATION "${_CONTROLLER_SDK_ROOT}/lib/libcontroller_sdk.so"
    IMPORTED_SONAME "libcontroller_sdk.so.1.3.1"
    INTERFACE_LINK_LIBRARIES "pthread;rt"
  )
endif()

set(ControllerSDK_LIBRARIES controller_sdk::controller_sdk)
set(ControllerSDK_INCLUDE_DIRS
  "${_CONTROLLER_SDK_ROOT}/include/controller_sdk"
  "${_CONTROLLER_SDK_ROOT}/include")
