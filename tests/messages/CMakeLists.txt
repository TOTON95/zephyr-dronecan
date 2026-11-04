# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.28.0)

# Use the module from this repository so the test builds standalone.
set(MODULE_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)
list(APPEND ZEPHYR_EXTRA_MODULES ${MODULE_DIR})

# The generated files must agree with each other: every message in the Kconfig options has an
# entry in the message table, and the other way round.
file(STRINGS ${MODULE_DIR}/src/dronecan_msg_table.c _table_lines REGEX "^\t\\{\"")
file(STRINGS ${MODULE_DIR}/Kconfig.messages _kconfig_lines REGEX "^config DRONECAN_MSG_")

# Messages that are always built have no option to turn on
file(READ ${MODULE_DIR}/Kconfig.messages _kconfig_text)
string(REGEX MATCHALL "config DRONECAN_MSG_[A-Z0-9_]+\n\tdef_bool y" _always_on "${_kconfig_text}")
string(REGEX REPLACE "config (DRONECAN_MSG_[A-Z0-9_]+)\n\tdef_bool y" "\\1" _always_on "${_always_on}")

set(_table_symbols "")
set(_kconfig_symbols "")
set(_conf "")
set(_includes "")
set(_expected "")

foreach(_line ${_table_lines})
  string(REGEX MATCH "\"([^\"]+)\"" _unused "${_line}")
  set(_name ${CMAKE_MATCH_1})
  string(REPLACE "." "_" _macro ${_name})
  string(TOUPPER ${_macro} _macro)
  list(APPEND _table_symbols DRONECAN_MSG_${_macro})

  # Turn the message on, and take its ID, signature and size from the generated header
  if(NOT "DRONECAN_MSG_${_macro}" IN_LIST _always_on)
    string(APPEND _conf "CONFIG_DRONECAN_MSG_${_macro}=y\n")
  endif()
  string(APPEND _includes "#include <${_name}.h>\n")
  string(APPEND _expected
    "\t{\"${_name}\", ${_macro}_ID, ${_macro}_SIGNATURE, ${_macro}_MAX_SIZE},\n")
endforeach()

foreach(_line ${_kconfig_lines})
  string(REPLACE "config " "" _symbol "${_line}")
  list(APPEND _kconfig_symbols ${_symbol})
endforeach()

list(SORT _table_symbols)
list(SORT _kconfig_symbols)
if(NOT "${_table_symbols}" STREQUAL "${_kconfig_symbols}")
  set(_only_table ${_table_symbols})
  list(REMOVE_ITEM _only_table ${_kconfig_symbols})
  set(_only_kconfig ${_kconfig_symbols})
  list(REMOVE_ITEM _only_kconfig ${_table_symbols})
  message(FATAL_ERROR
    "Kconfig.messages and src/dronecan_msg_table.c list different messages: run "
    "scripts/gen_kconfig_messages.py.\n"
    "  only in the table: ${_only_table}\n"
    "  only in Kconfig.messages: ${_only_kconfig}")
endif()

set(_gen ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${_gen})
file(WRITE ${_gen}/all_messages.conf "${_conf}")
file(WRITE ${_gen}/expected_includes.h "${_includes}")
file(WRITE ${_gen}/expected.inc "${_expected}")
list(APPEND EXTRA_CONF_FILE ${_gen}/all_messages.conf)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dronecan_messages)

# The table is only built with the shell: build it here instead
target_sources(app PRIVATE src/main.c ${MODULE_DIR}/src/dronecan_msg_table.c)
target_include_directories(app PRIVATE ${_gen} ${MODULE_DIR}/src)
