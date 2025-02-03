# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/sleepy/robp-group7-sleepy/build/micro_ros_agent/agent/src/xrceagent-build/spdlog/src/spdlog"
  "/home/sleepy/robp-group7-sleepy/build/micro_ros_agent/agent/src/xrceagent-build/spdlog/src/spdlog-build"
  "/home/sleepy/robp-group7-sleepy/build/micro_ros_agent/agent/src/xrceagent-build/temp_install/spdlog-1.9.2"
  "/home/sleepy/robp-group7-sleepy/build/micro_ros_agent/agent/src/xrceagent-build/spdlog/tmp"
  "/home/sleepy/robp-group7-sleepy/build/micro_ros_agent/agent/src/xrceagent-build/spdlog/src/spdlog-stamp"
  "/home/sleepy/robp-group7-sleepy/build/micro_ros_agent/agent/src/xrceagent-build/spdlog/src"
  "/home/sleepy/robp-group7-sleepy/build/micro_ros_agent/agent/src/xrceagent-build/spdlog/src/spdlog-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/sleepy/robp-group7-sleepy/build/micro_ros_agent/agent/src/xrceagent-build/spdlog/src/spdlog-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/sleepy/robp-group7-sleepy/build/micro_ros_agent/agent/src/xrceagent-build/spdlog/src/spdlog-stamp${cfgdir}") # cfgdir has leading slash
endif()
