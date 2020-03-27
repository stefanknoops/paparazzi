/*
 * Copyright (C) Stefan en de rest
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
 * @file "modules/farneback-avoider/farneback-avoider.h"
 * @author Stefan en de rest
 * Assignment 2019
 */
#ifndef TTC_CALCULATOR_H_
#define TTC_CALCULATOR_H_

extern void ttc_calc_init(void) ;
extern void ttc_calc_periodic(void);
extern float ttc_calculator_func();
struct image_t *image_func(struct image_t *img);



#endif
