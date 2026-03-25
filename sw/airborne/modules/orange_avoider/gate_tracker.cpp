/*
 * Gate Tracker - Blue banner detection and PnP distance estimation
 *
 * Adapted from Gate_Tracker.c standalone demo for use inside Paparazzi.
 * Receives YUYV422 frames from the video thread, detects two horizontal
 * blue banners arranged vertically (forming a gate), and uses OpenCV
 * solvePnP to estimate distance and horizontal offset.
 */

#include "modules/orange_avoider/gate_tracker.h"

#include <cmath>
#include <limits>
#include <vector>
#include <opencv2/opencv.hpp>

struct GateBanner {
  int cx, cy;
  int x, y, w, h;
};

int gate_tracker_process(char *img, int width, int height,
                         int32_t *quality, float *distance_m, float *offset_m)
{
  if (img == NULL || width <= 0 || height <= 0 ||
      quality == NULL || distance_m == NULL || offset_m == NULL) {
    return 0;
  }

  *quality    = 0;
  *distance_m = 0.f;
  *offset_m   = 0.f;

  /* ---- 1. Convert YUYV to BGR ---- */
  cv::Mat yuyv(height, width, CV_8UC2, img);
  cv::Mat bgr;
  cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUY2);

  /* ---- 2. Downscale to 320 wide ---- */
  int scaled_w = 320;
  int scaled_h = static_cast<int>(scaled_w * (static_cast<float>(height) / static_cast<float>(width)));
  if (scaled_h <= 0) {
    return 0;
  }

  cv::Mat small;
  cv::resize(bgr, small, cv::Size(scaled_w, scaled_h), 0.0, 0.0, cv::INTER_LINEAR);

  /* ---- 3. Blue colour mask in HSV ---- */
  cv::Mat hsv;
  cv::cvtColor(small, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(hsv, cv::Scalar(100, 100, 50), cv::Scalar(130, 255, 255), mask);

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

  *quality = cv::countNonZero(mask);

  /* ---- 4. Find contours and extract horizontal banners ---- */
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::vector<GateBanner> banners;
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

    float aspect = static_cast<float>(rect.width) / static_cast<float>(rect.height);
    if (aspect <= 1.2f) {
      continue;
    }

    cv::Moments m = cv::moments(contour);
    if (m.m00 == 0.0) {
      continue;
    }

    GateBanner b;
    b.cx = static_cast<int>(m.m10 / m.m00);
    b.cy = static_cast<int>(m.m01 / m.m00);
    b.x  = rect.x;
    b.y  = rect.y;
    b.w  = rect.width;
    b.h  = rect.height;
    banners.push_back(b);

    if (banners.size() >= 50) {
      break;
    }
  }

  if (banners.size() < 2) {
    return 0;
  }

  /* ---- 5. Pair banners vertically (small horizontal offset, large vertical gap) ---- */
  int best_i = -1, best_j = -1;
  float min_dx = std::numeric_limits<float>::max();

  for (size_t i = 0; i < banners.size(); i++) {
    for (size_t j = i + 1; j < banners.size(); j++) {
      float dx = std::fabs(static_cast<float>(banners[i].cx - banners[j].cx));
      float dy = std::fabs(static_cast<float>(banners[i].cy - banners[j].cy));

      if (dx < (static_cast<float>(banners[i].w) * 0.5f) &&
          dy > (static_cast<float>(banners[i].h) * 2.0f)) {
        if (dx < min_dx) {
          min_dx = dx;
          best_i = static_cast<int>(i);
          best_j = static_cast<int>(j);
        }
      }
    }
  }

  if (best_i < 0 || best_j < 0) {
    return 0;
  }

  /* ---- 6. Order top / bottom ---- */
  GateBanner top, bot;
  if (banners[best_i].y < banners[best_j].y) {
    top = banners[best_i];
    bot = banners[best_j];
  } else {
    top = banners[best_j];
    bot = banners[best_i];
  }

  /* ---- 7. Build image-point / object-point correspondences ---- */
  std::vector<cv::Point2f> img_pts = {
    cv::Point2f(static_cast<float>(top.x),         static_cast<float>(top.y)),
    cv::Point2f(static_cast<float>(top.x + top.w), static_cast<float>(top.y)),
    cv::Point2f(static_cast<float>(bot.x + bot.w), static_cast<float>(bot.y + bot.h)),
    cv::Point2f(static_cast<float>(bot.x),         static_cast<float>(bot.y + bot.h))
  };

  /* Gate real-world size: 1.5 m x 1.5 m  (half-extents 0.75 m) */
  std::vector<cv::Point3f> obj_pts = {
    cv::Point3f(-0.75f,  0.75f, 0.0f),
    cv::Point3f( 0.75f,  0.75f, 0.0f),
    cv::Point3f( 0.75f, -0.75f, 0.0f),
    cv::Point3f(-0.75f, -0.75f, 0.0f)
  };

  /* ---- 8. Camera intrinsics (approximate, for downscaled frame) ---- */
  double cx = static_cast<double>(scaled_w) / 2.0;
  double cy = static_cast<double>(scaled_h) / 2.0;

  cv::Mat K = (cv::Mat_<double>(3, 3) <<
    250.0, 0.0, cx,
    0.0, 250.0, cy,
    0.0,   0.0, 1.0);
  cv::Mat dist = cv::Mat::zeros(4, 1, CV_64F);

  /* ---- 9. Solve PnP ---- */
  cv::Mat rvec, tvec;
  bool ok = cv::solvePnP(obj_pts, img_pts, K, dist, rvec, tvec,
                          false, cv::SOLVEPNP_ITERATIVE);
  if (!ok || tvec.rows < 3) {
    return 0;
  }

  *offset_m   = static_cast<float>(tvec.at<double>(0, 0));
  *distance_m = static_cast<float>(tvec.at<double>(2, 0));

  return 1;
}
