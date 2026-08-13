add_library(a_star_search SHARED
  a_star_search.cc
)

pybind11_add_module(a_star
  python_interface.cc
)
target_link_libraries(a_star PRIVATE a_star_search)

if(BUILD_TESTING)
  add_executable(a_star_reset_regression_test
    test_search_reset.cc
  )
  target_link_libraries(a_star_reset_regression_test PRIVATE a_star_search)
  add_test(NAME a_star_reset_regression COMMAND a_star_reset_regression_test)
endif()
