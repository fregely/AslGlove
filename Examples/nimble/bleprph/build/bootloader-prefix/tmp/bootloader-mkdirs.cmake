# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/fregel/esp/esp-idf/components/bootloader/subproject"
  "/home/fregel/AslGlove/Examples/nimble/bleprph/build/bootloader"
  "/home/fregel/AslGlove/Examples/nimble/bleprph/build/bootloader-prefix"
  "/home/fregel/AslGlove/Examples/nimble/bleprph/build/bootloader-prefix/tmp"
  "/home/fregel/AslGlove/Examples/nimble/bleprph/build/bootloader-prefix/src/bootloader-stamp"
  "/home/fregel/AslGlove/Examples/nimble/bleprph/build/bootloader-prefix/src"
  "/home/fregel/AslGlove/Examples/nimble/bleprph/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/fregel/AslGlove/Examples/nimble/bleprph/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/fregel/AslGlove/Examples/nimble/bleprph/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
