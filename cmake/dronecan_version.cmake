# Copyright (c) 2026 3DR, Inc.
# SPDX-License-Identifier: Apache-2.0

# Obtain the version of the code in a git directory, for GetNodeInfo:
#  dronecan_git_version(<dir> <major> <minor> <commit>)

function(dronecan_git_version dir out_major out_minor out_commit)
  set(major 0)
  set(minor 0)
  set(commit 0)

  find_package(Git QUIET)
  if(GIT_FOUND AND IS_DIRECTORY ${dir})
    execute_process(
      COMMAND ${GIT_EXECUTABLE} -C ${dir} describe --tags --abbrev=0
              --match "[0-9]*" --match "v[0-9]*"
      OUTPUT_VARIABLE tag OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET RESULT_VARIABLE rc)
    if(rc EQUAL 0 AND tag MATCHES "^v?([0-9]+)\\.([0-9]+)")
      set(major ${CMAKE_MATCH_1})
      set(minor ${CMAKE_MATCH_2})
    endif()

    execute_process(
      COMMAND ${GIT_EXECUTABLE} -C ${dir} rev-parse HEAD
      OUTPUT_VARIABLE sha OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET RESULT_VARIABLE rc)
    if(rc EQUAL 0 AND sha MATCHES "^[0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f]")
      string(SUBSTRING ${sha} 0 8 short_sha)
      set(commit 0x${short_sha})
    endif()

    # Run CMake again when a commit is made
    execute_process(
      COMMAND ${GIT_EXECUTABLE} -C ${dir} rev-parse --absolute-git-dir
      OUTPUT_VARIABLE git_dir OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET RESULT_VARIABLE rc)
    if(rc EQUAL 0 AND EXISTS ${git_dir}/logs/HEAD)
      set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${git_dir}/logs/HEAD)
    endif()
  endif()

  set(${out_major} ${major} PARENT_SCOPE)
  set(${out_minor} ${minor} PARENT_SCOPE)
  set(${out_commit} ${commit} PARENT_SCOPE)
endfunction()
