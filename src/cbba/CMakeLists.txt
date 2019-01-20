# Include path
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(CBBA_LIB_SRC
    src/cbba_agent.cpp
    src/cbba_task.cpp
)

add_library(cbba STATIC ${CBBA_LIB_SRC})
target_link_libraries(cbba map ltl vis utility)
target_include_directories(cbba PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

add_executable(test_cbba test/test_cbba.cpp)
target_link_libraries(test_cbba cbba cbta vis ltl map utility ${OpenCV_LIBS})

add_executable(test_Kclosest test/test_Kclosest.cpp)
target_link_libraries(test_Kclosest cbba cbta vis ltl map utility ${OpenCV_LIBS})

add_executable(test_path_length test/test_path_length.cpp)
target_link_libraries(test_path_length cbba cbta vis ltl map utility ${OpenCV_LIBS})

add_executable(test_map test/test_map.cpp)
target_link_libraries(test_map cbba vis ltl map utility ${OpenCV_LIBS})

add_executable(test_syn test/test_syn.cpp)
target_link_libraries(test_syn cbba vis ltl map utility ${OpenCV_LIBS})

add_executable(test_tasks test/test_tasks.cpp)
target_link_libraries(test_tasks cbba vis ltl map utility ${OpenCV_LIBS})
