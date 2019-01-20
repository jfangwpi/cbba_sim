# Include path
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)

# Add libraries
set(LTL_LIB_SRC
    src/buchi_automaton.cpp
    src/spot_hoa_interpreter.cpp
    src/product_automaton.cpp
)

add_library(ltl STATIC ${LTL_LIB_SRC})
target_link_libraries(ltl map cbta spot bddx)
target_include_directories(ltl PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)


add_executable(test_buchi test/test_buchi.cpp)
target_link_libraries(test_buchi ltl)

add_executable(test_product test/test_product.cpp)
target_link_libraries(test_product vis ltl map ${OpenCV_LIBS})

add_executable(test_cosafe test/test_cosafe.cpp)
target_link_libraries(test_cosafe vis ltl map ${OpenCV_LIBS})

add_executable(test_multi_area test/test_multi_area.cpp)
target_link_libraries(test_multi_area vis ltl map ${OpenCV_LIBS})

add_executable(test_avoid_ROI test/test_avoid_ROI.cpp)
target_link_libraries(test_avoid_ROI vis ltl map ${OpenCV_LIBS})

