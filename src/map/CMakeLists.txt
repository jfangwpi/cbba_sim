# Include path
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)
find_package(PythonLibs 2.7)

# Add libraries
set(MAP_LIB_SRC
    src/square_grid.cpp  
)

add_library(map STATIC ${MAP_LIB_SRC})
target_link_libraries(map ltl vis utility)

target_include_directories(map PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)


add_executable(test_grid_map test/test_grid_map.cpp)
target_link_libraries(test_grid_map vis map utility ${OpenCV_LIBS})

add_executable(test_vis_lib test/test_vis_lib.cpp)
target_include_directories(test_vis_lib PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(test_vis_lib utility ${PYTHON_LIBRARIES})

add_executable(test_uncertain_grid_map test/test_uncertain_grid_map.cpp)
target_link_libraries(test_uncertain_grid_map vis map)


