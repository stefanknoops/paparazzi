/*
 * Copyright (C) Group 5
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/modulettc/modulettc.h"
 * @author Group 5
 * Calculates the time to contact from farneback optical flow vectors
 */

#ifndef MODULETTC_H
#define MODULETTC_H

extern float EWMA(float *history_ttc, int size_history, float degree_of_decrease);

extern void ttc_init();

extern void ttc_periodic();


#endif

