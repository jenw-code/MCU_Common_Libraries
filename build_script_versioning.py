import datetime
import os
Import("env")  # Required for PlatformIO

FILENAME_MINOR_BUILD_NO = 'versioning.log'
FILENAME_VERSION_H = 'lib/common/common.versioning.h'

MAJOR_VERSION_NO = 1
MINOR_VERSION_NO = 0
PATCH_VERSION_NO = 0
BUILD_DATE = datetime.datetime.today().strftime("%Y-%m-%d")

try:
    with open(FILENAME_MINOR_BUILD_NO) as f:
        f.readline()  # Skip comment line
        MAJOR_VERSION_NO = int(f.readline())
        MINOR_VERSION_NO = int(f.readline())
        PATCH_VERSION_NO = int(f.readline())
except:
    print('Error reading versioning file')

PATCH_VERSION_NO = PATCH_VERSION_NO + 1

with open(FILENAME_MINOR_BUILD_NO, 'w+') as f:
    f.write("** AUTO-EDITED - DO NOT CHANGE **\n")
    f.write(str(MAJOR_VERSION_NO)+'\n')
    f.write(str(MINOR_VERSION_NO)+'\n')
    f.write(str(PATCH_VERSION_NO))

print('Major version: {}'.format(MAJOR_VERSION_NO))
print('Minor version: {}'.format(MINOR_VERSION_NO))
print('Patch version: {}'.format(PATCH_VERSION_NO))

file_contents = f"""/* AUTO-GENERATED - DO NOT EDIT */
#pragma once
#include <arduino.h>

namespace VERSION {{
    inline constexpr uint16_t MAJOR = {MAJOR_VERSION_NO};
    inline constexpr uint16_t MINOR = {MINOR_VERSION_NO};
    inline constexpr uint16_t PATCH = {PATCH_VERSION_NO};
    inline constexpr char BUILD_DATE[] = \"{BUILD_DATE}\";
}}
"""
with open(FILENAME_VERSION_H, 'w+') as f:
    f.write(file_contents)
