/*
 * Copyright (C) 2019 Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/cv_detect_object.h
 * Assumes the object consists of a continuous color and checks
 * if you are over the defined object or not
 */

// Own header
#include "modules/computer_vision/cv_detect_color_object.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"

#define PRINT(string,...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;

#ifndef COLOR_OBJECT_DETECTOR_FPS1
#define COLOR_OBJECT_DETECTOR_FPS1 0 ///< Default FPS (zero means run at camera fps)
#endif
#ifndef COLOR_OBJECT_DETECTOR_FPS2
#define COLOR_OBJECT_DETECTOR_FPS2 0 ///< Default FPS (zero means run at camera fps)
#endif

// Filter Settings
uint8_t cod_lum_min1 = 0;
uint8_t cod_lum_max1 = 0;
uint8_t cod_cb_min1 = 0;
uint8_t cod_cb_max1 = 0;
uint8_t cod_cr_min1 = 0;
uint8_t cod_cr_max1 = 0;

//uint32_t orange_threshold = 

uint8_t cod_lum_min2 = 0;
uint8_t cod_lum_max2 = 0;
uint8_t cod_cb_min2 = 0;
uint8_t cod_cb_max2 = 0;
uint8_t cod_cr_min2 = 0;
uint8_t cod_cr_max2 = 0;

// GREEN
uint8_t cod_lum_min1_green = 0;
uint8_t cod_lum_max1_green = 0;
uint8_t cod_cb_min1_green = 0;
uint8_t cod_cb_max1_green = 0;
uint8_t cod_cr_min1_green = 0;
uint8_t cod_cr_max1_green = 0;

// uint32_t green_threshold = 8547;
uint32_t cod_green_threshold = 0;

// WHITE
uint8_t cod_lum_min1_white = 0;
uint8_t cod_lum_max1_white = 0;
uint8_t cod_cb_min1_white = 0;
uint8_t cod_cb_max1_white = 0;
uint8_t cod_cr_min1_white = 0;
uint8_t cod_cr_max1_white = 0;

// uint32_t white_threshold = 46248;
uint32_t cod_white_threshold = 0;

// BLACK
uint8_t cod_lum_min1_black = 0;
uint8_t cod_lum_max1_black = 0;
uint8_t cod_cb_min1_black = 0;
uint8_t cod_cb_max1_black = 0;
uint8_t cod_cr_min1_black = 0;
uint8_t cod_cr_max1_black = 0;

// uint32_t white_threshold = 46248;
uint32_t cod_black_threshold = 0;

// RAINBOW
uint8_t cod_lum_min1_rainbow = 0;
uint8_t cod_lum_max1_rainbow = 0;
uint8_t cod_cb_min1_rainbow = 0;
uint8_t cod_cb_max1_rainbow = 0;
uint8_t cod_cr_min1_rainbow = 0;
uint8_t cod_cr_max1_rainbow = 0;

// uint32_t white_threshold = 46248;
uint32_t cod_rainbow_threshold = 0;


bool cod_draw1 = false;
bool cod_draw2 = false;
bool cod_draw_white = false;
bool cod_draw_green = false;
bool cod_draw_black = false;
bool cod_draw_rainbow = false;

// define global variables
struct color_object_t {
  int32_t x_c;
  int32_t y_c;
  uint32_t color_count;
  bool updated;
};
struct color_object_t global_filters[2];

// Function
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max);

/*
 * object_detector
 * @param img - input image to process
 * @param filter - which detection filter to process
 * @return img
 */
static struct image_t *object_detector(struct image_t *img, uint8_t filter)
{
  uint8_t lum_min, lum_max;
  uint8_t cb_min, cb_max;
  uint8_t cr_min, cr_max;
  uint8_t lum_min_w, lum_max_w;
  uint8_t cb_min_w, cb_max_w;
  uint8_t cr_min_w, cr_max_w;
  uint8_t lum_min_g, lum_max_g;
  uint8_t cb_min_g, cb_max_g;
  uint8_t cr_min_g, cr_max_g;
  uint8_t lum_min_b, lum_max_b;
  uint8_t cb_min_b, cb_max_b;
  uint8_t cr_min_b, cr_max_b;
  uint8_t lum_min_rb, lum_max_rb;
  uint8_t cb_min_rb, cb_max_rb;
  uint8_t cr_min_rb, cr_max_rb;
  uint32_t white_threshold;
  uint32_t green_threshold;
  uint32_t black_threshold;
  uint32_t rainbow_threshold;
  bool draw;
  bool draw_white;
  bool draw_green;
  bool draw_black;
  bool draw_rainbow;

  switch (filter){
    case 1:
      lum_min = cod_lum_min1;
      lum_max = cod_lum_max1;
      cb_min = cod_cb_min1;
      cb_max = cod_cb_max1;
      cr_min = cod_cr_min1;
      cr_max = cod_cr_max1;
      draw = cod_draw1;

      // White
      lum_min_w = cod_lum_min1_white;
      lum_max_w = cod_lum_max1_white;
      cb_min_w = cod_cb_min1_white;
      cb_max_w = cod_cb_max1_white;
      cr_min_w = cod_cr_min1_white;
      cr_max_w = cod_cr_max1_white;
      draw_white = cod_draw_white;
      white_threshold = cod_white_threshold;

      // Green
      lum_min_g = cod_lum_min1_green;
      lum_max_g = cod_lum_max1_green;
      cb_min_g = cod_cb_min1_green;
      cb_max_g = cod_cb_max1_green;
      cr_min_g = cod_cr_min1_green;
      cr_max_g = cod_cr_max1_green;
      draw_green = cod_draw_green;
      green_threshold = cod_green_threshold;

      // Black
      lum_min_b = cod_lum_min1_black;
      lum_max_b = cod_lum_max1_black;
      cb_min_b = cod_cb_min1_black;
      cb_max_b = cod_cb_max1_black;
      cr_min_b = cod_cr_min1_black;
      cr_max_b = cod_cr_max1_black;
      draw_black = cod_draw_black;
      black_threshold = cod_black_threshold;

      // Rainbow
      lum_min_rb = cod_lum_min1_rainbow;
      lum_max_rb = cod_lum_max1_rainbow;
      cb_min_rb = cod_cb_min1_rainbow;
      cb_max_rb = cod_cb_max1_rainbow;
      cr_min_rb = cod_cr_min1_rainbow;
      cr_max_rb = cod_cr_max1_rainbow;
      draw_rainbow = cod_draw_rainbow;
      rainbow_threshold = cod_rainbow_threshold;
      break;
    case 2:
      lum_min = cod_lum_min2;
      lum_max = cod_lum_max2;
      cb_min = cod_cb_min2;
      cb_max = cod_cb_max2;
      cr_min = cod_cr_min2;
      cr_max = cod_cr_max2;
      draw = cod_draw2;

      // White
      lum_min_w = cod_lum_min1_white;
      lum_max_w = cod_lum_max1_white;
      cb_min_w = cod_cb_min1_white;
      cb_max_w = cod_cb_max1_white;
      cr_min_w = cod_cr_min1_white;
      cr_max_w = cod_cr_max1_white;
      draw_white = cod_draw_white;
      white_threshold = cod_white_threshold;

      // Green
      lum_min_g = cod_lum_min1_green;
      lum_max_g = cod_lum_max1_green;
      cb_min_g = cod_cb_min1_green;
      cb_max_g = cod_cb_max1_green;
      cr_min_g = cod_cr_min1_green;
      cr_max_g = cod_cr_max1_green;
      draw_green = cod_draw_green;
      green_threshold = cod_green_threshold;

      // Black
      lum_min_b = cod_lum_min1_black;
      lum_max_b = cod_lum_max1_black;
      cb_min_b = cod_cb_min1_black;
      cb_max_b = cod_cb_max1_black;
      cr_min_b = cod_cr_min1_black;
      cr_max_b = cod_cr_max1_black;
      draw_black = cod_draw_black;
      black_threshold = cod_black_threshold;

      // Rainbow
      lum_min_rb = cod_lum_min1_rainbow;
      lum_max_rb = cod_lum_max1_rainbow;
      cb_min_rb = cod_cb_min1_rainbow;
      cb_max_rb = cod_cb_max1_rainbow;
      cr_min_rb = cod_cr_min1_rainbow;
      cr_max_rb = cod_cr_max1_rainbow;
      draw_rainbow = cod_draw_rainbow;
      rainbow_threshold = cod_rainbow_threshold;
      break;
    default:
      return img;
  };

  int32_t x_c, y_c;
  int32_t x_c_white, y_c_white;
  int32_t x_c_green, y_c_green;
  int32_t x_c_black, y_c_black;
  int32_t x_c_rainbow, y_c_rainbow;

  // Filter and find centroid
  uint32_t count_orange = find_object_centroid(img, &x_c, &y_c, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max);
  uint32_t count_green = find_object_centroid(img, &x_c_green, &y_c_green, draw_green, lum_min_g, lum_max_g, cb_min_g, cb_max_g, cr_min_g, cr_max_g);
  uint32_t count_white = find_object_centroid(img, &x_c_white, &y_c_white, draw_white, lum_min_w, lum_max_w, cb_min_w, cb_max_w, cr_min_w, cr_max_w);
  uint32_t count_black = find_object_centroid(img, &x_c_black, &y_c_black, draw_black, lum_min_b, lum_max_b, cb_min_b, cb_max_b, cr_min_b, cr_max_b);
  uint32_t count_rainbow = find_object_centroid(img, &x_c_rainbow, &y_c_rainbow, draw_rainbow, lum_min_rb, lum_max_rb, cb_min_rb, cb_max_rb, cr_min_rb, cr_max_rb);
  
  
  VERBOSE_PRINT("Color count %d: %u, threshold %u, x_c %d, y_c %d\n", camera, object_count, count_threshold, x_c, y_c);
  VERBOSE_PRINT("White count %d: %u, threshold %u, x_c %d, y_c %d\n", camera, object_count_white, count_threshold_white, x_c_white, y_c_white);
  VERBOSE_PRINT("Green count %d: %u, threshold %u, x_c %d, y_c %d\n", camera, object_count_green, count_threshold_green, x_c_green, y_c_green);
  VERBOSE_PRINT("centroid %d: (%d, %d) r: %4.2f a: %4.2f\n", camera, x_c, y_c,
        hypotf(x_c, y_c) / hypotf(img->w * 0.5, img->h * 0.5), RadOfDeg(atan2f(y_c, x_c)));
        
  uint8_t threshold_surpassed = 0; // number of colors surpassing the thresholds
  if (count_green > green_threshold) threshold_surpassed++;
  if (count_white > white_threshold) threshold_surpassed++;
  if (count_black > black_threshold) threshold_surpassed++;
  if (count_rainbow > rainbow_threshold) threshold_surpassed++;
  
  uint32_t count = 0;
  if (threshold_surpassed > 0) count = 100000;

  pthread_mutex_lock(&mutex);
  global_filters[filter-1].color_count = count; // 0: no critical color counts, 100000: at least one threshold surpassed
  global_filters[filter-1].x_c = x_c;
  global_filters[filter-1].y_c = y_c;
  global_filters[filter-1].updated = true;
  pthread_mutex_unlock(&mutex);

  return img;
}

struct image_t *object_detector1(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 1);
}

struct image_t *object_detector2(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector2(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 2);
}

void color_object_detector_init(void)
{
  memset(global_filters, 0, 2*sizeof(struct color_object_t));
  pthread_mutex_init(&mutex, NULL);
#ifdef COLOR_OBJECT_DETECTOR_CAMERA1
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN1
  // Orange
  cod_lum_min1 = COLOR_OBJECT_DETECTOR_LUM_MIN1;
  cod_lum_max1 = COLOR_OBJECT_DETECTOR_LUM_MAX1;
  cod_cb_min1 = COLOR_OBJECT_DETECTOR_CB_MIN1;
  cod_cb_max1 = COLOR_OBJECT_DETECTOR_CB_MAX1;
  cod_cr_min1 = COLOR_OBJECT_DETECTOR_CR_MIN1;
  cod_cr_max1 = COLOR_OBJECT_DETECTOR_CR_MAX1;
  // White
  cod_lum_min1_white = COLOR_OBJECT_DETECTOR_LUM_MIN1_WHITE;
  cod_lum_max1_white = COLOR_OBJECT_DETECTOR_LUM_MAX1_WHITE;
  cod_cb_min1_white = COLOR_OBJECT_DETECTOR_CB_MIN1_WHITE;
  cod_cb_max1_white = COLOR_OBJECT_DETECTOR_CB_MAX1_WHITE;
  cod_cr_min1_white = COLOR_OBJECT_DETECTOR_CR_MIN1_WHITE;
  cod_cr_max1_white = COLOR_OBJECT_DETECTOR_CR_MAX1_WHITE;
  // Green
  cod_lum_min1_green = COLOR_OBJECT_DETECTOR_LUM_MIN1_GREEN;
  cod_lum_max1_green = COLOR_OBJECT_DETECTOR_LUM_MAX1_GREEN;
  cod_cb_min1_green = COLOR_OBJECT_DETECTOR_CB_MIN1_GREEN;
  cod_cb_max1_green = COLOR_OBJECT_DETECTOR_CB_MAX1_GREEN;
  cod_cr_min1_green = COLOR_OBJECT_DETECTOR_CR_MIN1_GREEN;
  cod_cr_max1_green = COLOR_OBJECT_DETECTOR_CR_MAX1_GREEN;
    // Black
  cod_lum_min1_black = COLOR_OBJECT_DETECTOR_LUM_MIN1_BLACK;
  cod_lum_max1_black = COLOR_OBJECT_DETECTOR_LUM_MAX1_BLACK;
  cod_cb_min1_black = COLOR_OBJECT_DETECTOR_CB_MIN1_BLACK;
  cod_cb_max1_black = COLOR_OBJECT_DETECTOR_CB_MAX1_BLACK;
  cod_cr_min1_black = COLOR_OBJECT_DETECTOR_CR_MIN1_BLACK;
  cod_cr_max1_black = COLOR_OBJECT_DETECTOR_CR_MAX1_BLACK;
  // Rainbow
  cod_lum_min1_rainbow = COLOR_OBJECT_DETECTOR_LUM_MIN1_RAINBOW;
  cod_lum_max1_rainbow = COLOR_OBJECT_DETECTOR_LUM_MAX1_RAINBOW;
  cod_cb_min1_rainbow = COLOR_OBJECT_DETECTOR_CB_MIN1_RAINBOW;
  cod_cb_max1_rainbow = COLOR_OBJECT_DETECTOR_CB_MAX1_RAINBOW;
  cod_cr_min1_rainbow = COLOR_OBJECT_DETECTOR_CR_MIN1_RAINBOW;
  cod_cr_max1_rainbow = COLOR_OBJECT_DETECTOR_CR_MAX1_RAINBOW;

  cod_green_threshold = COLOR_OBJECT_DETECTOR_GREEN_THRESHOLD;
  cod_white_threshold = COLOR_OBJECT_DETECTOR_WHITE_THRESHOLD;
  cod_black_threshold = COLOR_OBJECT_DETECTOR_BLACK_THRESHOLD;
  cod_rainbow_threshold = COLOR_OBJECT_DETECTOR_RAINBOW_THRESHOLD;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW1
  cod_draw1 = COLOR_OBJECT_DETECTOR_DRAW1;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, object_detector1, COLOR_OBJECT_DETECTOR_FPS1, 0);
#endif

#ifdef COLOR_OBJECT_DETECTOR_CAMERA2
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN2
  cod_lum_min2 = COLOR_OBJECT_DETECTOR_LUM_MIN2;
  cod_lum_max2 = COLOR_OBJECT_DETECTOR_LUM_MAX2;
  cod_cb_min2 = COLOR_OBJECT_DETECTOR_CB_MIN2;
  cod_cb_max2 = COLOR_OBJECT_DETECTOR_CB_MAX2;
  cod_cr_min2 = COLOR_OBJECT_DETECTOR_CR_MIN2;
  cod_cr_max2 = COLOR_OBJECT_DETECTOR_CR_MAX2;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW2
  cod_draw2 = COLOR_OBJECT_DETECTOR_DRAW2;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA2, object_detector2, COLOR_OBJECT_DETECTOR_FPS2, 1);
#endif
}

/*
 * find_object_centroid
 *
 * Finds the centroid of pixels in an image within filter bounds.
 * Also returns the amount of pixels that satisfy these filter bounds.
 *
 * @param img - input image to process formatted as YUV422.
 * @param p_xc - x coordinate of the centroid of color object
 * @param p_yc - y coordinate of the centroid of color object
 * @param lum_min - minimum y value for the filter in YCbCr colorspace
 * @param lum_max - maximum y value for the filter in YCbCr colorspace
 * @param cb_min - minimum cb value for the filter in YCbCr colorspace
 * @param cb_max - maximum cb value for the filter in YCbCr colorspace
 * @param cr_min - minimum cr value for the filter in YCbCr colorspace
 * @param cr_max - maximum cr value for the filter in YCbCr colorspace
 * @param draw - whether or not to draw on image
 * @return number of pixels of image within the filter bounds.
 */
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max)
{
  uint32_t cnt = 0;
  uint32_t tot_x = 0;
  uint32_t tot_y = 0;
  uint8_t *buffer = img->buf;

  // Go through all the pixels
  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 80; x < img->w-80; x ++) {
      // Check if the color is inside the specified values
      uint8_t *yp, *up, *vp;
      if (x % 2 == 0) {
        // Even x
        up = &buffer[y * 2 * img->w + 2 * x];      // U
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
        vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
        //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
      } else {
        // Uneven x
        up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
        //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
        vp = &buffer[y * 2 * img->w + 2 * x];      // V
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
      }
      if ( (*yp >= lum_min) && (*yp <= lum_max) &&
           (*up >= cb_min ) && (*up <= cb_max ) &&
           (*vp >= cr_min ) && (*vp <= cr_max )) {
        cnt ++;
        tot_x += x;
        tot_y += y;
        if (draw){
          *yp = 255;  // make pixel brighter in image
        }
      }
    }
  }
  if (cnt > 0) {
    *p_xc = (int32_t)roundf(tot_x / ((float) cnt) - img->w * 0.5f);
    *p_yc = (int32_t)roundf(img->h * 0.5f - tot_y / ((float) cnt));
  } else {
    *p_xc = 0;
    *p_yc = 0;
  }
  return cnt;
}

void color_object_detector_periodic(void)
{
  static struct color_object_t local_filters[2];
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, 2*sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);

  if(local_filters[0].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].x_c, local_filters[0].y_c,
        0, 0, local_filters[0].color_count, 0);
    local_filters[0].updated = false;
  }
  if(local_filters[1].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID, local_filters[1].x_c, local_filters[1].y_c,
        0, 0, local_filters[1].color_count, 1);
    local_filters[1].updated = false;
  }
}