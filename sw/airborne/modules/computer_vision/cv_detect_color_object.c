/*
 * Copyright (C) 2019 Kirk Scheper <kirkscheper@gmail.com>
 *
 * Modified to support ROI masking for filter 1:
 *   - filter 1: lower trapezoid ROI only
 *   - filter 2: full frame
 */

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
#define COLOR_OBJECT_DETECTOR_FPS1 0
#endif
#ifndef COLOR_OBJECT_DETECTOR_FPS2
#define COLOR_OBJECT_DETECTOR_FPS2 0
#endif

uint8_t cod_lum_min1 = 0;
uint8_t cod_lum_max1 = 0;
uint8_t cod_cb_min1 = 0;
uint8_t cod_cb_max1 = 0;
uint8_t cod_cr_min1 = 0;
uint8_t cod_cr_max1 = 0;

uint8_t cod_lum_min2 = 0;
uint8_t cod_lum_max2 = 0;
uint8_t cod_cb_min2 = 0;
uint8_t cod_cb_max2 = 0;
uint8_t cod_cr_min2 = 0;
uint8_t cod_cr_max2 = 0;

bool cod_draw1 = false;
bool cod_draw2 = false;

struct color_object_t {
  int32_t x_c;
  int32_t y_c;
  uint32_t color_count;
  bool updated;
};
struct color_object_t global_filters[2];

/* New helpers */
static bool pixel_in_lower_trapezoid(uint16_t x, uint16_t y, uint16_t img_w, uint16_t img_h);
static bool pixel_on_lower_trapezoid_border(uint16_t x, uint16_t y, uint16_t img_w, uint16_t img_h);

uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max,
                              uint8_t filter);

/*
 * Lower trapezoid ROI:
 *   x from 0 to 0.45*img_w
 *   top margin decreases linearly from 60 to 0.20*img_h
 *   bottom margin mirrors the top
 */
static bool pixel_in_lower_trapezoid(uint16_t x, uint16_t y, uint16_t img_w, uint16_t img_h)
{
  int col_end = (int)(0.45f * img_w);
  int margin_start = 60;
  int margin_end = (int)(0.20f * img_h);

  if (col_end <= 0) {
    return false;
  }

  if ((int)x < 0 || (int)x > col_end) {
    return false;
  }

  float alpha = (float)x / (float)col_end;
  float margin = (1.0f - alpha) * margin_start + alpha * margin_end;

  int y_min = (int)roundf(margin);
  int y_max = (int)roundf((float)img_h - margin);

  return ((int)y >= y_min && (int)y <= y_max);
}

static bool pixel_on_lower_trapezoid_border(uint16_t x, uint16_t y, uint16_t img_w, uint16_t img_h)
{
  int col_end = (int)(0.45f * img_w);
  int margin_start = 60;
  int margin_end = (int)(0.20f * img_h);

  if (col_end <= 0) {
    return false;
  }

  if ((int)x < 0 || (int)x > col_end) {
    return false;
  }

  float alpha = (float)x / (float)col_end;
  float margin = (1.0f - alpha) * margin_start + alpha * margin_end;

  int y_min = (int)roundf(margin);
  int y_max = (int)roundf((float)img_h - margin);

  /* border thickness = 1 pixel */
  if ((int)x == 0 || (int)x == col_end) {
    if ((int)y >= y_min && (int)y <= y_max) {
      return true;
    }
  }

  if (abs((int)y - y_min) <= 1 || abs((int)y - y_max) <= 1) {
    return true;
  }

  return false;
}

static struct image_t *object_detector(struct image_t *img, uint8_t filter)
{
  uint8_t lum_min, lum_max;
  uint8_t cb_min, cb_max;
  uint8_t cr_min, cr_max;
  bool draw;

  switch (filter){
    case 1:
      lum_min = cod_lum_min1;
      lum_max = cod_lum_max1;
      cb_min = cod_cb_min1;
      cb_max = cod_cb_max1;
      cr_min = cod_cr_min1;
      cr_max = cod_cr_max1;
      draw = cod_draw1;
      break;
    case 2:
      lum_min = cod_lum_min2;
      lum_max = cod_lum_max2;
      cb_min = cod_cb_min2;
      cb_max = cod_cb_max2;
      cr_min = cod_cr_min2;
      cr_max = cod_cr_max2;
      draw = cod_draw2;
      break;
    default:
      return img;
  };

  int32_t x_c, y_c;

  uint32_t count = find_object_centroid(img, &x_c, &y_c, draw,
                                        lum_min, lum_max, cb_min, cb_max, cr_min, cr_max,
                                        filter);

  pthread_mutex_lock(&mutex);
  global_filters[filter-1].color_count = count;
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
  cod_lum_min1 = COLOR_OBJECT_DETECTOR_LUM_MIN1;
  cod_lum_max1 = COLOR_OBJECT_DETECTOR_LUM_MAX1;
  cod_cb_min1 = COLOR_OBJECT_DETECTOR_CB_MIN1;
  cod_cb_max1 = COLOR_OBJECT_DETECTOR_CB_MAX1;
  cod_cr_min1 = COLOR_OBJECT_DETECTOR_CR_MIN1;
  cod_cr_max1 = COLOR_OBJECT_DETECTOR_CR_MAX1;
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

uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max,
                              uint8_t filter)
{
  uint32_t cnt = 0;
  uint32_t tot_x = 0;
  uint32_t tot_y = 0;
  uint8_t *buffer = img->buf;

  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 0; x < img->w; x++) {

      bool inside_roi = true;

      /* filter 1 (orange): only count pixels inside trapezoid */
      if (filter == 1) {
        inside_roi = pixel_in_lower_trapezoid(x, y, img->w, img->h);
      }

      uint8_t *yp, *up, *vp;
      if (x % 2 == 0) {
        up = &buffer[y * 2 * img->w + 2 * x];
        yp = &buffer[y * 2 * img->w + 2 * x + 1];
        vp = &buffer[y * 2 * img->w + 2 * x + 2];
      } else {
        up = &buffer[y * 2 * img->w + 2 * x - 2];
        vp = &buffer[y * 2 * img->w + 2 * x];
        yp = &buffer[y * 2 * img->w + 2 * x + 1];
      }

      /* draw trapezoid border for filter 1 */
      if (draw && filter == 1 && pixel_on_lower_trapezoid_border(x, y, img->w, img->h)) {
        *yp = 255;
      }

      if (!inside_roi) {
        continue;
      }

      if ((*yp >= lum_min) && (*yp <= lum_max) &&
          (*up >= cb_min)  && (*up <= cb_max)  &&
          (*vp >= cr_min)  && (*vp <= cr_max)) {
        cnt++;
        tot_x += x;
        tot_y += y;
        if (draw) {
          *yp = 255;
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
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID,
                               local_filters[0].x_c, local_filters[0].y_c,
                               0, 0, local_filters[0].color_count, 0);
    local_filters[0].updated = false;
  }
  if(local_filters[1].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID,
                               local_filters[1].x_c, local_filters[1].y_c,
                               0, 0, local_filters[1].color_count, 1);
    local_filters[1].updated = false;
  }
}
