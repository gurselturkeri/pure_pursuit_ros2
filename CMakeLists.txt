cmake_minimum_required(VERSION 3.5)
project(pure_pursuit_ros2)

# C++ standardı
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ROS2 ile uyumlu hale gelmek için gerekli paketler
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Derlenecek kaynak dosyalar
add_executable(pure_pursuit_node 
src/main.cpp
src/pure_pursuit_node.cpp)

# Bağımlı olunan hedefler
ament_target_dependencies(pure_pursuit_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
)

# Kurulum ayarları
install(TARGETS
  pure_pursuit_node
  DESTINATION lib/${PROJECT_NAME}
)

# Parametre dosyası vb. gibi yapılandırma dosyalarını kur
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

include_directories(include)

install(DIRECTORY include/
  DESTINATION include/
)

target_include_directories(pure_pursuit_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_package()
