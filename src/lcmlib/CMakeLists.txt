# Include path
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)

## Find LCM
find_package(LCM REQUIRED)

if(LCM_NOT_FOUND)
  message(STATUS "LCM is not found. Inter-process communication depends on LCM.")
endif(LCM_NOT_FOUND)


# Add libraries
set(LCMLIB_LIB_SRC
    src/msg_exchange.cpp
)

add_library(lcmlib STATIC ${LCMLIB_LIB_SRC})
target_link_libraries(lcmlib cbba spot map bddx)
target_include_directories(lcmlib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

#add_executable(test_pi1 test/test_pi1.cpp)
#target_link_libraries(test_pi1 lcmlib ltl map vis cbba ${LCM_LIBRARIES})

#add_executable(test_pi2 test/test_pi2.cpp)
#target_link_libraries(test_pi2 lcmlib ltl map vis cbba ${LCM_LIBRARIES})

#add_executable(test_pi1_cbta test/test_pi1_cbta.cpp)
#target_link_libraries(test_pi1_cbta lcmlib ltl map vis cbba ${LCM_LIBRARIES})

#add_executable(test_pi2_cbta test/test_pi2_cbta.cpp)
#target_link_libraries(test_pi2_cbta lcmlib ltl map vis cbba ${LCM_LIBRARIES})

#add_executable(test_pi3_cbta test/test_pi3_cbta.cpp)
#target_link_libraries(test_pi3_cbta lcmlib ltl map vis cbba ${LCM_LIBRARIES})

#add_executable(test_pi3 test/test_pi3.cpp)
#target_link_libraries(test_pi3 lcmlib ltl map vis cbba ${LCM_LIBRARIES})

#add_executable(publisher test/publisher.cpp)
#target_link_libraries(publisher lcmlib ltl map vis cbba ${LCM_LIBRARIES})

#add_executable(subscriber test/subscriber.cpp)
#target_link_libraries(subscriber lcmlib ltl map vis cbba ${LCM_LIBRARIES})

#add_executable(subscribe_motor test/subscribe_motor.cpp)
#target_link_libraries(subscribe_motor lcmlib ltl map vis cbba ${LCM_LIBRARIES})

#add_executable(publish_motor test/publish_motor.cpp)
#target_link_libraries(publish_motor lcmlib ltl map vis cbba ${LCM_LIBRARIES})

#add_executable(test_gpio test/test_gpio.cpp)
#target_link_libraries(test_gpio lcmlib ltl map vis cbba ${LCM_LIBRARIES})

#add_executable(test_pi4 test/test_pi4.cpp)
#target_link_libraries(test_pi4 lcmlib ltl map vis cbba ${LCM_LIBRARIES})
