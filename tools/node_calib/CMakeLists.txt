set (CMAKE_CXX_STANDARD 14)

add_executable(node_calib main.cpp)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(GTK3 REQUIRED IMPORTED_TARGET gtk+-3.0)

target_link_libraries(node_calib PRIVATE
    k4a::k4a
    ${OpenCV_LIBS}
    PkgConfig::GTK3
    pthread
    k4ainternal::dynlib)

