# Include path
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(CBGA_LIB_SRC
    src/cbga_agent.cpp
    src/cbga_task.cpp
)

add_library(cbga STATIC ${CBGA_LIB_SRC})
target_link_libraries(cbga cbba map ltl vis utility)
target_include_directories(cbga PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

add_executable(test_cbga test/test_cbga.cpp)
target_link_libraries(test_cbga cbga cbta cbba vis ltl map utility ${OpenCV_LIBS})

add_executable(test_cbga_wt test/test_cbga_wt.cpp)
target_link_libraries(test_cbga_wt cbga cbta cbba vis ltl map utility ${OpenCV_LIBS})

add_executable(test_cbga_cbta test/test_cbga_cbta.cpp)
target_link_libraries(test_cbga_cbta cbta cbba cbga vis map ltl utility ${OpenCV_LIBS})
