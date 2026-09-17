include_guard(GLOBAL)

if(TARGET vhit_cia402_core::vhit_cia402_core)
  return()
endif()

# Prefer a system-installed copy.
find_package(vhit_cia402_core 0.0.2 CONFIG QUIET)

if(TARGET vhit_cia402_core::vhit_cia402_core)
  message(STATUS "Using installed vhit_cia402_core")
  return()
endif()

option(
  VHIT_CIA402_FETCH_IF_MISSING
  "Download vhit_cia402_core when it is not installed"
  ON
)

if(NOT VHIT_CIA402_FETCH_IF_MISSING)
  message(FATAL_ERROR
    "vhit_cia402_core 0.0.2 was not found. "
    "Install it or enable VHIT_CIA402_FETCH_IF_MISSING."
  )
endif()

include(FetchContent)

message(STATUS "vhit_cia402_core not installed; fetching it")

FetchContent_Declare(
  vhit_cia402_core
  GIT_REPOSITORY
    https://github.com/NotValcake/VHIT-CiA402-Core.git
  GIT_TAG
    v0.0.2
  GIT_PROGRESS TRUE
)

FetchContent_MakeAvailable(vhit_cia402_core)

# The fetched core defaults to a static library. It must be PIC because it is
# linked into a shared ROS 2 plugin.
set_target_properties(vhit_cia402_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

if(NOT TARGET vhit_cia402_core::vhit_cia402_core)
  message(FATAL_ERROR
    "vhit_cia402_core was fetched but did not define its expected CMake target"
  )
endif()
