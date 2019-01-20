# Include path
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)

# Add libraries
set(CBTA_LIB_SRC
    src/graph_lift.cpp
    src/hcost_interface.cpp
    src/defn_hlevel.cpp
    src/defn_tile_block.cpp
    src/defn_tile.cpp
)

add_library(cbta STATIC ${CBTA_LIB_SRC})
target_link_libraries(cbta map spot bddx utility)
target_include_directories(cbta PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)


add_executable(test_lifted_graph test/test_lifted_graph.cpp)
target_link_libraries(test_lifted_graph cbta vis map ${OpenCV_LIBS})

add_executable(test_cbta test/test_cbta.cpp)
target_link_libraries(test_cbta cbta vis map ltl ${OpenCV_LIBS})

add_executable(test_path_length_cbta test/test_path_length_cbta.cpp)
target_link_libraries(test_path_length_cbta cbba cbta vis ltl map utility ${OpenCV_LIBS})

add_executable(test_cbba_cbta test/test_cbba_cbta.cpp)
target_link_libraries(test_cbba_cbta cbta cbba vis map ltl ${OpenCV_LIBS})

add_executable(test_syn_cbta test/test_syn_cbta.cpp)
target_link_libraries(test_syn_cbta cbta cbba vis map ltl ${OpenCV_LIBS})