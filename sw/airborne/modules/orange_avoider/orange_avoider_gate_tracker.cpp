/*
 * Copyright (C) Paparazzi Team
 *
 * This file is part of paparazzi
 *
 */

#include "modules/orange_avoider/orange_avoider_gate_tracker.h"

#include <cmath>
#include <limits>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

struct OaBanner {
  int cx;
  int cy;
  int x;
  int y;
  int w;
  int h;
};

int orange_avoider_gate_tracker_process(char *img, int width, int height,
                                        int32_t *quality, float *distance_m, float *offset_m)
{
  if (img == nullptr || width <= 0 || height <= 0 || quality == nullptr || distance_m == nullptr || offset_m == nullptr) {
    return 0;
  }

  cv::Mat yuyv(height, width, CV_8UC2, img);
  cv::Mat bgr;
  cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUY2);

  int scaled_width = 320;
  int scaled_height = static_cast<int>(scaled_width * (static_cast<float>(height) / static_cast<float>(width)));
  if (scaled_height <= 0) {
    return 0;
  }

  cv::Mat small;
  cv::resize(bgr, small, cv::Size(scaled_width, scaled_height), 0.0, 0.0, cv::INTER_LINEAR);

  cv::Mat hsv;
  cv::cvtColor(small, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(hsv, cv::Scalar(100, 100, 50), cv::Scalar(130, 255, 255), mask);

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

  *quality = cv::countNonZero(mask);
  *distance_m = 0.f;
  *offset_m = 0.f;

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::vector<OaBanner> banners;
  banners.reserve(50);

  for (const auto &contour : contours) {
    double area = cv::contourArea(contour);
    if (area <= 100.0) {
      continue;
    }

    cv::Rect rect = cv::boundingRect(contour);
    if (rect.height <= 0) {
      continue;
    }

    float aspect_ratio = static_cast<float>(rect.width) / static_cast<float>(rect.height);
    if (aspect_ratio <= 1.2f) {
      continue;
    }

    cv::Moments m = cv::moments(contour);
    if (m.m00 == 0.0) {
      continue;
    }

    OaBanner b;
    b.cx = static_cast<int>(m.m10 / m.m00);
    b.cy = static_cast<int>(m.m01 / m.m00);
    b.x = rect.x;
    b.y = rect.y;
    b.w = rect.width;
    b.h = rect.height;
    banners.push_back(b);

    if (banners.size() >= 50) {
      break;
    }
  }

  if (banners.size() < 2) {
    return 0;
  }

  int best_i = -1;
  int best_j = -1;
  float min_offset_x = std::numeric_limits<float>::max();

  for (size_t i = 0; i < banners.size(); i++) {
    for (size_t j = i + 1; j < banners.size(); j++) {
      const OaBanner &b1 = banners[i];
      const OaBanner &b2 = banners[j];

      float dx = std::fabs(static_cast<float>(b1.cx - b2.cx));
      float dy = std::fabs(static_cast<float>(b1.cy - b2.cy));

      if (dx < (static_cast<float>(b1.w) * 0.5f) && dy > (static_cast<float>(b1.h) * 2.0f)) {
        if (dx < min_offset_x) {
          min_offset_x = dx;
          best_i = static_cast<int>(i);
          best_j = static_cast<int>(j);
        }
      }
    }
  }

  if (best_i < 0 || best_j < 0) {
    return 0;
  }

  OaBanner top_banner;
  OaBanner bot_banner;
  if (banners[best_i].y < banners[best_j].y) {
    top_banner = banners[best_i];
    bot_banner = banners[best_j];
  } else {
    top_banner = banners[best_j];
    bot_banner = banners[best_i];
  }

  std::vector<cv::Point2f> image_points = {
    cv::Point2f(static_cast<float>(top_banner.x), static_cast<float>(top_banner.y)),
    cv::Point2f(static_cast<float>(top_banner.x + top_banner.w), static_cast<float>(top_banner.y)),
    cv::Point2f(static_cast<float>(bot_banner.x + bot_banner.w), static_cast<float>(bot_banner.y + bot_banner.h)),
    cv::Point2f(static_cast<float>(bot_banner.x), static_cast<float>(bot_banner.y + bot_banner.h))
  };

  std::vector<cv::Point3f> object_points = {
    cv::Point3f(-0.75f,  0.75f, 0.0f),
    cv::Point3f( 0.75f,  0.75f, 0.0f),
    cv::Point3f( 0.75f, -0.75f, 0.0f),
    cv::Point3f(-0.75f, -0.75f, 0.0f)
  };

  cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
    250.0, 0.0, 160.0,
    0.0, 250.0, 120.0,
    0.0, 0.0, 1.0);
  cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);
  cv::Mat rvec, tvec;

  bool solved = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec, false,
                             cv::SOLVEPNP_ITERATIVE);
  if (!solved || tvec.rows < 3) {
    return 0;
  }

  *offset_m = static_cast<float>(tvec.at<double>(0, 0));
  *distance_m = static_cast<float>(tvec.at<double>(2, 0));

  return 1;
}
