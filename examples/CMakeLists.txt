#
# (C) Copyright 2009-2021 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawMaxonEPOSExamples VERSION 1.1.0)

set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstCommonQt
                              cisstVector
                              cisstNumerical
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes
                              cisstParameterTypesQt
                              cisstQt)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.1.0 COMPONENTS ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawMaxonEPOSExamples_IS_CATKIN_BUILT)
  if (sawMaxonEPOSExamples_IS_CATKIN_BUILT)
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
  endif ()

  # sawMaxonEPOS has been compiled within cisst, we should find it automatically
  find_package (sawMaxonEPOS REQUIRED)

  if (sawMaxonEPOS_FOUND)

    # sawMaxonEPOS configuration
    include_directories (${sawMaxonEPOS_INCLUDE_DIR})
    link_directories (${sawMaxonEPOS_LIBRARY_DIR})

    add_executable (sawMaxonEPOSQtExample maxonUIControl.cpp)
    set_target_properties (sawMaxonEPOSQtExample PROPERTIES
                           COMPONENT sawMaxonEPOS-Examples
                           FOLDER "sawMaxonEPOS")

    # link against non cisst libraries and saw components
    target_link_libraries (sawMaxonEPOSQtExample
                           ${sawMaxonEPOS_LIBRARIES}
                           ${sawMaxonEPOSQt_LIBRARIES})

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (sawMaxonEPOSQtExample ${REQUIRED_CISST_LIBRARIES})

    install (TARGETS sawMaxonEPOSQtExample COMPONENT sawMaxonEPOS-Examples
             RUNTIME DESTINATION bin
             LIBRARY DESTINATION lib
             ARCHIVE DESTINATION lib)

  endif (sawMaxonEPOS_FOUND)

endif (cisst_FOUND_AS_REQUIRED)
