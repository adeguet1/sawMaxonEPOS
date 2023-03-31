#
# (C) Copyright 2011-2021 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawMaxonEPOS VERSION 1.0.0)

set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstVector
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

find_package (cisst 1.1.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawMaxonEPOS_IS_CATKIN_BUILT)
  if (sawMaxonEPOS_IS_CATKIN_BUILT)
    set (sawMaxonEPOS_CONFIG_FILE_DIR
         "${CATKIN_DEVEL_PREFIX}/share/sawMaxonEPOS/cmake")
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
    set (LIBRARY_OUTPUT_PATH    "${CATKIN_DEVEL_PREFIX}/lib")
  else ()
    if (NOT LIBRARY_OUTPUT_PATH)
      set (EXECUTABLE_OUTPUT_PATH "${CMAKE_CURRENT_BINARY_DIR}/bin")
      set (LIBRARY_OUTPUT_PATH "${CMAKE_CURRENT_BINARY_DIR}/lib")
    endif (NOT LIBRARY_OUTPUT_PATH)
    set (sawMaxonEPOS_CONFIG_FILE_DIR "${sawMaxonEPOS_BINARY_DIR}")
  endif ()

  # find EPOS library, installer also create a link in /usr/lib
  set (KNOWN_VERSIONS 6.6.2.0 6.8.1.0)
  set (EPOS_SEARCH_PATH "")
  foreach (vers ${KNOWN_VERSIONS})
    set (EPOS_SEARCH_PATH "${EPOS_SEARCH_PATH}" "/opt/EposCmdLib_${vers}")
  endforeach ()
  find_library (EPOS_LIBRARY EposCmd
                PATHS ${EPOS_SEARCH_PATH})
  find_path (EPOS_INCLUDE_DIR "Definitions.h"
             PATHS ${EPOS_SEARCH_PATH}
             PATH_SUFFIXES "include")

  # create/configure file for find_package (sawMaxonEPOS)
  set (sawMaxonEPOS_INCLUDE_DIR
       "${sawMaxonEPOS_SOURCE_DIR}/include"
       "${sawMaxonEPOS_BINARY_DIR}/include")
  set (sawMaxonEPOS_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawMaxonEPOS_LIBRARIES sawMaxonEPOS ${EPOS_LIBRARY})
  set (sawMaxonEPOSQt_LIBRARIES sawMaxonEPOSQt)

  # Generate sawMaxonEPOSRevision.h
  configure_file ("${sawMaxonEPOS_SOURCE_DIR}/code/sawMaxonEPOSRevision.h.in"
                  "${sawMaxonEPOS_BINARY_DIR}/include/sawMaxonEPOS/sawMaxonEPOSRevision.h")

  include_directories (${sawMaxonEPOS_INCLUDE_DIR} ${sawMaxonEPOS_BINARY_DIR} ${EPOS_INCLUDE_DIR})
  set (sawMaxonEPOS_HEADER_DIR "${sawMaxonEPOS_SOURCE_DIR}/include/sawMaxonEPOS")

  add_library (sawMaxonEPOS
               ${sawMaxonEPOS_HEADER_DIR}/sawMaxonEPOSExport.h
               code/maxonInterface.cpp
               ${sawMaxonEPOS_HEADER_DIR}/maxonInterface.h
               code/maxonMotor.cpp
               ${sawMaxonEPOS_HEADER_DIR}/maxonMotor.h
               code/maxonMotorInterface.cpp
               ${sawMaxonEPOS_HEADER_DIR}/maxonMotorInterface.h)
  set_target_properties (sawMaxonEPOS PROPERTIES
                         VERSION ${sawMaxonEPOS_VERSION}
                         FOLDER "sawMaxonEPOS")
  cisst_target_link_libraries (sawMaxonEPOS ${REQUIRED_CISST_LIBRARIES})

  # add Qt code
  if (CISST_HAS_QT)
    add_subdirectory (code/Qt)
    set (sawMaxonEPOS_LIBRARIES ${sawMaxonEPOS_LIBRARIES} ${sawMaxonEPOSQt_LIBRARIES})
  endif (CISST_HAS_QT)

  set (sawMaxonEPOS_CMAKE_CONFIG_FILE
       "${sawMaxonEPOS_CONFIG_FILE_DIR}/sawMaxonEPOSConfig.cmake")

  configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/cmake/sawMaxonEPOSConfig.cmake.in"
                  "${sawMaxonEPOS_CMAKE_CONFIG_FILE}"
                  @ONLY)

  cisst_add_config_version ("${sawMaxonEPOS_CONFIG_FILE_DIR}/sawMaxonEPOSConfigVersion.cmake"
                            VERSION ${sawMaxonEPOS_VERSION}
                            DESTINATION "share/sawMaxonEPOS"
                            COMPONENT sawMaxonEPOS)

  # Install target for headers and library
  install (DIRECTORY
           "${sawMaxonEPOS_SOURCE_DIR}/include/sawMaxonEPOS"
           "${sawMaxonEPOS_BINARY_DIR}/include/sawMaxonEPOS"
           DESTINATION include
           COMPONENT sawMaxonEPOS-dev
           PATTERN .svn EXCLUDE)

  install (TARGETS sawMaxonEPOS COMPONENT sawMaxonEPOS
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

  install (FILES "${sawMaxonEPOS_CMAKE_CONFIG_FILE}"
           DESTINATION "share/sawMaxonEPOS"
           COMPONENT sawMaxonEPOS-dev)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
