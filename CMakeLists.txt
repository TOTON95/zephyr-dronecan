# Copyright (c) 2026 3DR, Inc.
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_DRONECAN)

  zephyr_library()

  # libcanard
  set(LIBCANARD_DIR ${ZEPHYR_CURRENT_MODULE_DIR}/libcanard)
  if(NOT EXISTS ${LIBCANARD_DIR}/canard.c)
    message(FATAL_ERROR
      "libcanard submodule is missing. Set 'submodules: true' on the zephyr-dronecan west project, or run 'git submodule update --init'.")
  endif()

  # DSDL + Generator (as west projects, not Zephyr modules)
  # They are looked for in <workspace>/lib: first in the workspace of the Zephyr tree being built,
  # then in the workspace of the application. Applications outside the workspace work in the
  # first case. It can be overridden with -DDRONECAN_DSDL_DIR=... and -DDRONECAN_DSDLC_DIR=...
  set(_dsdl_lib_candidates ${ZEPHYR_BASE}/../lib)
  if(WEST_TOPDIR)
    list(APPEND _dsdl_lib_candidates ${WEST_TOPDIR}/lib)
  endif()

  set(_dsdl_lib ${ZEPHYR_BASE}/../lib) # used in the error below when nothing matches
  foreach(_c ${_dsdl_lib_candidates})
    if(IS_DIRECTORY ${_c}/DSDL AND IS_DIRECTORY ${_c}/dronecan_dsdlc)
      set(_dsdl_lib ${_c})
      break()
    endif()
  endforeach()

  set(DRONECAN_DSDL_DIR  ${_dsdl_lib}/DSDL            CACHE PATH "DroneCAN DSDL definitions")
  set(DRONECAN_DSDLC_DIR ${_dsdl_lib}/dronecan_dsdlc  CACHE PATH "dronecan_dsdlc generator")

  foreach(_d ${DRONECAN_DSDL_DIR} ${DRONECAN_DSDLC_DIR})
    if(NOT IS_DIRECTORY ${_d})
      message(FATAL_ERROR "${_d} not found. Looked for DSDL and dronecan_dsdlc in: ${_dsdl_lib_candidates}. Add the DSDL and dronecan_dsdlc west projects to your manifest and run 'west update', or pass -DDRONECAN_DSDL_DIR=... and -DDRONECAN_DSDLC_DIR=...")
    endif()
  endforeach()

  # Collect DSDL namespaces defined in Kconfig
  set(_ns_roots "")
  if(CONFIG_DRONECAN_DSDL_UAVCAN)
    list(APPEND _ns_roots ${DRONECAN_DSDL_DIR}/uavcan)
  endif()
  if(CONFIG_DRONECAN_DSDL_DRONECAN)
    list(APPEND _ns_roots ${DRONECAN_DSDL_DIR}/dronecan)
  endif()
  if(CONFIG_DRONECAN_DSDL_ARDUPILOT)
    list(APPEND _ns_roots ${DRONECAN_DSDL_DIR}/ardupilot)
  endif()
  if(NOT _ns_roots)
    message(FATAL_ERROR "Enable at least one CONFIG_DRONECAN_DSDL_* namespace.")
  endif()

  # Generation of C code
  set(_gen_dir   ${CMAKE_CURRENT_BINARY_DIR}/dsdlc_generated)
  set(_stamp     ${_gen_dir}/.stamp)
  file(GLOB_RECURSE _dsdl_files ${DRONECAN_DSDL_DIR}/*.uavcan)

  # Keep CMake pending of any definition changes
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${_dsdl_files})

  # Regenerate if the namespace set changed or any input is newer than the stamp
  set(_regen FALSE)
  if(NOT EXISTS ${_stamp})
    set(_regen TRUE)
  else()
    file(READ ${_stamp} _stamp_ns)
    if(NOT "${_stamp_ns}" STREQUAL "${_ns_roots}")
      set(_regen TRUE)
    endif()
    foreach(_f ${_dsdl_files})
      if(${_f} IS_NEWER_THAN ${_stamp})
        set(_regen TRUE)
        break()
      endif()
    endforeach()
  endif()

  if(_regen)
    message(STATUS "DroneCAN: generating DSDL code (${_ns_roots})")
    file(REMOVE_RECURSE ${_gen_dir})
    execute_process(
      COMMAND ${Python3_EXECUTABLE} ${DRONECAN_DSDLC_DIR}/dronecan_dsdlc.py -O ${_gen_dir} ${_ns_roots}
      RESULT_VARIABLE _rc
    )
    if(NOT _rc EQUAL 0)
      message(FATAL_ERROR "dronecan_dsdlc failed (exited with ${_rc}), check if dependencies are installed")
    endif()
    file(WRITE ${_stamp} "${_ns_roots}")
  endif()

  # Build only the messages enabled in Kconfig. Types without a DRONECAN_MSG_* symbol (nested
  # types and services) are always built.
  file(STRINGS ${CMAKE_CURRENT_LIST_DIR}/Kconfig.messages _msg_lines REGEX "^config DRONECAN_MSG_")
  string(REGEX REPLACE "config " "" _msg_symbols "${_msg_lines}")
  file(GLOB _dsdl_all_srcs ${_gen_dir}/src/*.c)
  set(_dsdl_gen_srcs "")
  foreach(_src ${_dsdl_all_srcs})
    get_filename_component(_type ${_src} NAME)
    string(REGEX REPLACE "\\.c$" "" _type ${_type})
    string(TOUPPER ${_type} _type)
    string(REPLACE "." "_" _sym DRONECAN_MSG_${_type})
    if(NOT _sym IN_LIST _msg_symbols OR CONFIG_${_sym})
      list(APPEND _dsdl_gen_srcs ${_src})
    endif()
  endforeach()

  # Build
  zephyr_include_directories(
    include
    ${LIBCANARD_DIR}
    ${_gen_dir}/include
  )

  zephyr_library_sources(
    ${LIBCANARD_DIR}/canard.c
    ${_dsdl_gen_srcs}
    src/dronecan.c
  )

  zephyr_library_sources_ifdef(CONFIG_DRONECAN_SHELL src/dronecan_shell.c)

endif() # CONFIG_DRONECAN
