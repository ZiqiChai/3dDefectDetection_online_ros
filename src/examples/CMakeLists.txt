# Build example 1
ADD_EXECUTABLE(example_1 example_1.c)
TARGET_LINK_LIBRARIES(example_1 ${GOCATOR_LIBRARIES} ${KAPI_LIBRARIES})

# Build example 2
ADD_EXECUTABLE(example_2 example_2.c)
TARGET_LINK_LIBRARIES(example_2 ${GOCATOR_LIBRARIES} ${KAPI_LIBRARIES})

# Build example 3
ADD_EXECUTABLE(example_3 example_3.cpp)
TARGET_LINK_LIBRARIES(example_3 ${GOCATOR_LIBRARIES} ${KAPI_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# Build example 4
ADD_EXECUTABLE(example_4 example_4.cpp)
TARGET_LINK_LIBRARIES(example_4 ${GOCATOR_LIBRARIES} ${KAPI_LIBRARIES} ${PCL_LIBRARIES} ${PROJECT_NAME}_LIB)

# Build example_5_factory_restore
ADD_EXECUTABLE(example_5 example_5_factory_restore.c)
TARGET_LINK_LIBRARIES(example_5 ${GOCATOR_LIBRARIES} ${KAPI_LIBRARIES})

# Build example_6_health_temperature
ADD_EXECUTABLE(example_6 example_6_health_temperature.cpp)
TARGET_LINK_LIBRARIES(example_6 ${GOCATOR_LIBRARIES} ${KAPI_LIBRARIES} ${PCL_LIBRARIES} ${PROJECT_NAME}_LIB)
