# Install script for directory: /home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/src/ur10e_hl_interface

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/install/ur10e_hl_interface")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/build/ur10e_hl_interface/libur10e_hl_interface.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur10e_hl_interface/clipfix_bewegung" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur10e_hl_interface/clipfix_bewegung")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur10e_hl_interface/clipfix_bewegung"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ur10e_hl_interface" TYPE EXECUTABLE FILES "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/build/ur10e_hl_interface/clipfix_bewegung")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur10e_hl_interface/clipfix_bewegung" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur10e_hl_interface/clipfix_bewegung")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur10e_hl_interface/clipfix_bewegung"
         OLD_RPATH "/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur10e_hl_interface/clipfix_bewegung")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur10e_hl_interface/add_objects_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur10e_hl_interface/add_objects_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur10e_hl_interface/add_objects_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ur10e_hl_interface" TYPE EXECUTABLE FILES "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/build/ur10e_hl_interface/add_objects_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur10e_hl_interface/add_objects_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur10e_hl_interface/add_objects_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur10e_hl_interface/add_objects_node"
         OLD_RPATH "/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur10e_hl_interface/add_objects_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/src/ur10e_hl_interface/include/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur10e_hl_interface/config" TYPE DIRECTORY FILES "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/src/ur10e_hl_interface/config/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur10e_hl_interface/meshes" TYPE DIRECTORY FILES "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/src/ur10e_hl_interface/meshes/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur10e_hl_interface/environment" TYPE FILE FILES "/opt/ros/humble/lib/python3.10/site-packages/ament_package/template/environment_hook/library_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur10e_hl_interface/environment" TYPE FILE FILES "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/build/ur10e_hl_interface/ament_cmake_environment_hooks/library_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/build/ur10e_hl_interface/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/ur10e_hl_interface")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/build/ur10e_hl_interface/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/ur10e_hl_interface")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur10e_hl_interface/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur10e_hl_interface/environment" TYPE FILE FILES "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/build/ur10e_hl_interface/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur10e_hl_interface/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur10e_hl_interface/environment" TYPE FILE FILES "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/build/ur10e_hl_interface/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur10e_hl_interface" TYPE FILE FILES "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/build/ur10e_hl_interface/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur10e_hl_interface" TYPE FILE FILES "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/build/ur10e_hl_interface/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur10e_hl_interface" TYPE FILE FILES "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/build/ur10e_hl_interface/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur10e_hl_interface" TYPE FILE FILES "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/build/ur10e_hl_interface/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur10e_hl_interface" TYPE FILE FILES "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/build/ur10e_hl_interface/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/build/ur10e_hl_interface/ament_cmake_index/share/ament_index/resource_index/packages/ur10e_hl_interface")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ur10e_hl_interface/cmake/export_ur10e_hl_interfaceExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ur10e_hl_interface/cmake/export_ur10e_hl_interfaceExport.cmake"
         "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/build/ur10e_hl_interface/CMakeFiles/Export/share/ur10e_hl_interface/cmake/export_ur10e_hl_interfaceExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ur10e_hl_interface/cmake/export_ur10e_hl_interfaceExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ur10e_hl_interface/cmake/export_ur10e_hl_interfaceExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur10e_hl_interface/cmake" TYPE FILE FILES "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/build/ur10e_hl_interface/CMakeFiles/Export/share/ur10e_hl_interface/cmake/export_ur10e_hl_interfaceExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur10e_hl_interface/cmake" TYPE FILE FILES "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/build/ur10e_hl_interface/CMakeFiles/Export/share/ur10e_hl_interface/cmake/export_ur10e_hl_interfaceExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur10e_hl_interface/cmake" TYPE FILE FILES "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/build/ur10e_hl_interface/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur10e_hl_interface/cmake" TYPE FILE FILES "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/build/ur10e_hl_interface/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur10e_hl_interface/cmake" TYPE FILE FILES
    "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/build/ur10e_hl_interface/ament_cmake_core/ur10e_hl_interfaceConfig.cmake"
    "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/build/ur10e_hl_interface/ament_cmake_core/ur10e_hl_interfaceConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur10e_hl_interface" TYPE FILE FILES "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/src/ur10e_hl_interface/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/levin/Semi-Automated-Programming-of-Industrial-Robotic-Systems-Using-Large-Language-Models/build/ur10e_hl_interface/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
