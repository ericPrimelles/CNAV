add_executable(test main.cpp)
target_link_libraries( test "${TORCH_LIBRARIES}" RVO )
set_property(TARGET test PROPERTY CXX_STANDARD 17)