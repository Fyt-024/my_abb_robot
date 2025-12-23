cmake_minimum_required(VERSION 3.8)
project(my_abb_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# 1. 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# 2. 安装 urdf 文件
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

# 3. [新增] 安装 config 文件 (这是之前缺少的！)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# 4. 安装 rviz 配置文件
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

# 5. [新增] 安装 Python 脚本 (让您可以通过 ros2 run 运行它)
install(PROGRAMS
  scripts/move_robot.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

