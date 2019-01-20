# Include path
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)
find_package(MathGL2 2.4)
find_package(PythonLibs 2.7)



# Add libraries
set(VIS_LIB_SRC
    src/vis_utils.cpp
    src/graph_vis.cpp
)

add_library(vis STATIC ${VIS_LIB_SRC})
target_link_libraries(vis map utility ${OpenCV_LIBS} ${PYTHON_LIBRARIES})
target_include_directories(vis PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)


include_directories(
    ${MathGL_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
)





