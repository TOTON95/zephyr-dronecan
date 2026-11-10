# Copyright (c) 2026 3DR, Inc.
# SPDX-License-Identifier: Apache-2.0

# Test of dronecan_git_version(), with repositories made on the fly:
#   cmake -P  cmake/test_version.cmake

# needed to use dronecan_git_version()
include(${CMAKE_CURRENT_LIST_DIR}/dronecan_version.cmake)

string(RANDOM LENGTH 8 suffix)
if(DEFINED ENV{TMPDIR})
  set(work $ENV{TMPDIR}/dronecan_version_test_${suffix})
else()
  set(work /tmp/dronecan_version_test_${suffix})
endif()

file(MAKE_DIRECTORY ${work})

find_package(Git REQUIRED)

macro(git)
  execute_process(COMMAND ${GIT_EXECUTABLE} -C ${repo} ${ARGN}
    OUTPUT_QUIET ERROR_VARIABLE git_err RESULT_VARIABLE git_rc)
  if(NOT git_rc EQUAL 0)
    message(FATAL_ERROR "git ${ARGN} failed: ${git_err}")
  endif()
endmacro()

function(expect what major minor has_commit)
  dronecan_git_version(${repo} got_major got_minor got_commit)
  if(NOT got_major EQUAL ${major} OR NOT got_minor EQUAL ${minor})
    message(FATAL_ERROR "${what}: got ${got_major}.${got_minor}, expected ${major}.${minor}")
  endif()
  if(has_commit AND got_commit STREQUAL "0")
    message(FATAL_ERROR "${what}: no commit")
  endif()
  if(NOT has_commit AND NOT got_commit STREQUAL "0")
    message(FATAL_ERROR "${what}: unexpected commit ${got_commit}")
  endif()
  message(STATUS "ok: ${what}: ${got_major}.${got_minor}, commit ${got_commit}")
endfunction()

set(repo ${work}/plain)
file(MAKE_DIRECTORY ${repo})
expect("not a git directory" 0 0 FALSE)

set(repo ${work}/repo)
file(MAKE_DIRECTORY ${repo})
git(init -q)
git(config user.email test@example.com)
git(config user.name corgi)
git(config commit.gpgsign false)
git(commit -q --allow-empty -m "woof x1")
expect("no tag" 0 0 TRUE)

git(tag v1.2.3)
expect("tag v1.2.3" 1 2 TRUE)

git(commit -q --allow-empty -m "woof x2")
expect("commit after the tag" 1 2 TRUE)

git(tag 2.5)
expect("tag 2.5, without v" 2 5 TRUE)

git(commit -q --allow-empty -m "woof x3")
git(tag v3.4-rc1)
expect("tag with a suffix" 3 4 TRUE)

git(commit -q --allow-empty -m "bark!")
git(tag nightly)
expect("a tag that is not a version should be ignored" 3 4 TRUE)

file(REMOVE_RECURSE ${work})
message(STATUS "all dronecan_git_version tests passed!")
