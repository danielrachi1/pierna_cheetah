# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/danielrachi/esp/v5.3.1/esp-idf/components/bootloader/subproject"
  "/home/danielrachi/Documents/pierna_cheetah/prototipos/can_basico/twai_self_test/build/bootloader"
  "/home/danielrachi/Documents/pierna_cheetah/prototipos/can_basico/twai_self_test/build/bootloader-prefix"
  "/home/danielrachi/Documents/pierna_cheetah/prototipos/can_basico/twai_self_test/build/bootloader-prefix/tmp"
  "/home/danielrachi/Documents/pierna_cheetah/prototipos/can_basico/twai_self_test/build/bootloader-prefix/src/bootloader-stamp"
  "/home/danielrachi/Documents/pierna_cheetah/prototipos/can_basico/twai_self_test/build/bootloader-prefix/src"
  "/home/danielrachi/Documents/pierna_cheetah/prototipos/can_basico/twai_self_test/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/danielrachi/Documents/pierna_cheetah/prototipos/can_basico/twai_self_test/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/danielrachi/Documents/pierna_cheetah/prototipos/can_basico/twai_self_test/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
