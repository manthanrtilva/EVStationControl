
add_executable(SingleStationWidget SingleStationWidget.cpp)
target_link_libraries(SingleStationWidget
  ${GTKMM_LIBRARIES}
)

add_executable(DynamicStationAdd DynamicStationAdd.cpp)
target_link_libraries(DynamicStationAdd
  ${GTKMM_LIBRARIES}
)

add_executable(DisplayLandingPage DisplayLandingPage.cpp)
target_link_libraries(DisplayLandingPage
  ${GTKMM_LIBRARIES}
)
