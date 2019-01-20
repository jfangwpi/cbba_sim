# Include path
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})


# Add libraries
set(UTILITY_LIB_SRC
)

add_library(utility INTERFACE)
target_include_directories(utility INTERFACE include/)
