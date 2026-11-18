/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_WEIGHT_SENSOR_ENABLED
#if HAL_PROGRAM_SIZE_LIMIT_KB <= 1024
#define AP_WEIGHT_SENSOR_ENABLED 0
#else
#define AP_WEIGHT_SENSOR_ENABLED 1
#endif
#endif
