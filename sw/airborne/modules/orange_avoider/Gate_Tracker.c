#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/calib3d/calib3d_c.h>

namespace fs = std::filesystem;

// Struct to hold our banner data
typedef struct {
    int cx, cy;
    int x, y, w, h;
} Banner;

bool has_image_extension(const fs::path& path) {
    std::string ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return (char)std::tolower(c); });
    return ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp";
}

bool is_numeric_name(const fs::path& path, long long& value) {
    std::string stem = path.stem().string();
    if (stem.empty()) return false;
    for (char c : stem) {
        if (!std::isdigit((unsigned char)c)) return false;
    }
    value = std::stoll(stem);
    return true;
}

std::vector<fs::path> get_sorted_frame_files(const std::string& folder_path) {
    std::vector<fs::path> files;

    if (!fs::exists(folder_path) || !fs::is_directory(folder_path)) {
        return files;
    }

    for (const auto& entry : fs::directory_iterator(folder_path)) {
        if (entry.is_regular_file() && has_image_extension(entry.path())) {
            files.push_back(entry.path());
        }
    }

    std::sort(files.begin(), files.end(), [](const fs::path& a, const fs::path& b) {
        long long va = 0, vb = 0;
        bool a_is_num = is_numeric_name(a, va);
        bool b_is_num = is_numeric_name(b, vb);

        if (a_is_num && b_is_num) return va < vb;
        if (a_is_num != b_is_num) return a_is_num;
        return a.filename().string() < b.filename().string();
    });

    return files;
}

void two_step_pnp_tracker_c(IplImage* frame, bool show_window, cv::Mat* output_frame) {
    // 1. Downscale
    int height = frame->height;
    int width = frame->width;
    int new_width = 320;
    int new_height = (int)(new_width * ((float)height / width));

    IplImage* small_frame = cvCreateImage(cvSize(new_width, new_height), frame->depth, frame->nChannels);
    cvResize(frame, small_frame, CV_INTER_LINEAR);

    // 2. Strong Blue Masking
    IplImage* hsv = cvCreateImage(cvGetSize(small_frame), 8, 3);
    cvCvtColor(small_frame, hsv, CV_BGR2HSV);

    IplImage* mask = cvCreateImage(cvGetSize(small_frame), 8, 1);
    CvScalar lower_blue = cvScalar(100, 100, 50, 0); // H, S, V
    CvScalar upper_blue = cvScalar(130, 255, 255, 0);
    cvInRangeS(hsv, lower_blue, upper_blue, mask);

    // 3. Clean up the mask
    IplConvKernel* kernel = cvCreateStructuringElementEx(5, 5, 2, 2, CV_SHAPE_RECT, NULL);
    // Note: cvMorphologyEx in the C API requires a temporary image
    IplImage* temp = cvCreateImage(cvGetSize(small_frame), 8, 1);
    cvMorphologyEx(mask, mask, temp, kernel, CV_MOP_OPEN, 1);
    cvMorphologyEx(mask, mask, temp, kernel, CV_MOP_CLOSE, 1);

    // 4. Find valid Blue Banners
    CvMemStorage* storage = cvCreateMemStorage(0);
    CvSeq* contours = 0;
    cvFindContours(mask, storage, &contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));

    Banner banners[50];
    int num_banners = 0;

    for (CvSeq* c = contours; c != NULL; c = c->h_next) {
        double area = cvContourArea(c, CV_WHOLE_SEQ, 0);
        if (area > 100.0) {
            CvRect rect = cvBoundingRect(c, 0);
            float aspect_ratio = (float)rect.width / (float)rect.height;
            
            if (aspect_ratio > 1.2f) {
                CvMoments moments;
                cvMoments(c, &moments, 0);
                if (moments.m00 != 0) {
                    banners[num_banners].cx = (int)(moments.m10 / moments.m00);
                    banners[num_banners].cy = (int)(moments.m01 / moments.m00);
                    banners[num_banners].x = rect.x;
                    banners[num_banners].y = rect.y;
                    banners[num_banners].w = rect.width;
                    banners[num_banners].h = rect.height;
                    
                    cvDrawContours(small_frame, c, cvScalar(0, 165, 255, 0), cvScalar(0, 165, 255, 0), 0, CV_FILLED, 8, cvPoint(0,0));
                    num_banners++;
                    if (num_banners >= 50) break; // Prevent array overflow
                }
            }
        }
    }

    // 5. VERTICAL PAIRING LOGIC
    if (num_banners >= 2) {
        Banner* b1_best = NULL;
        Banner* b2_best = NULL;
        float min_offset_x = 999999.0f;

        for (int i = 0; i < num_banners; i++) {
            for (int j = i + 1; j < num_banners; j++) {
                Banner* b1 = &banners[i];
                Banner* b2 = &banners[j];
                
                float dx = (float)fabs(b1->cx - b2->cx);
                float dy = (float)fabs(b1->cy - b2->cy);
                
                if (dx < (b1->w * 0.5f) && dy > (b1->h * 2.0f)) {
                    if (dx < min_offset_x) {
                        min_offset_x = dx;
                        b1_best = b1;
                        b2_best = b2;
                    }
                }
            }
        }

        if (b1_best != NULL && b2_best != NULL) {
            Banner top_banner, bot_banner;
            if (b1_best->y < b2_best->y) {
                top_banner = *b1_best;
                bot_banner = *b2_best;
            } else {
                top_banner = *b2_best;
                bot_banner = *b1_best;
            }

            // 6. EXTRACT THE 4 CORNERS FOR 3D MATH
            float img_pts[4][2] = {
                {(float)top_banner.x, (float)top_banner.y},
                {(float)(top_banner.x + top_banner.w), (float)top_banner.y},
                {(float)(bot_banner.x + bot_banner.w), (float)(bot_banner.y + bot_banner.h)},
                {(float)bot_banner.x, (float)(bot_banner.y + bot_banner.h)}
            };
            CvMat img_mat = cvMat(4, 2, CV_32FC1, img_pts);

            for (int i = 0; i < 4; i++) {
                cvCircle(small_frame, cvPoint((int)img_pts[i][0], (int)img_pts[i][1]), 6, cvScalar(0, 0, 255, 0), -1, 8, 0);
            }

            // 7. SOLVE PnP (OpenCV4-compatible API)
            float obj_pts[4][3] = {
                {-0.75f,  0.75f, 0.0f},
                { 0.75f,  0.75f, 0.0f},
                { 0.75f, -0.75f, 0.0f},
                {-0.75f, -0.75f, 0.0f}
            };
            std::vector<cv::Point3f> object_points = {
                cv::Point3f(obj_pts[0][0], obj_pts[0][1], obj_pts[0][2]),
                cv::Point3f(obj_pts[1][0], obj_pts[1][1], obj_pts[1][2]),
                cv::Point3f(obj_pts[2][0], obj_pts[2][1], obj_pts[2][2]),
                cv::Point3f(obj_pts[3][0], obj_pts[3][1], obj_pts[3][2])
            };
            std::vector<cv::Point2f> image_points = {
                cv::Point2f(img_pts[0][0], img_pts[0][1]),
                cv::Point2f(img_pts[1][0], img_pts[1][1]),
                cv::Point2f(img_pts[2][0], img_pts[2][1]),
                cv::Point2f(img_pts[3][0], img_pts[3][1])
            };

            cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
                250.0, 0.0, 160.0,
                0.0, 250.0, 120.0,
                0.0, 0.0, 1.0);
            cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);
            cv::Mat rvec, tvec;

            cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

            // Output telemetry
            float horizontal_offset = (float)tvec.at<double>(0, 0);
            float distance_to_gate = (float)tvec.at<double>(2, 0);
            
            char text_dist[50], text_off[50];
            sprintf(text_dist, "Dist: %.2fm", distance_to_gate);
            sprintf(text_off, "Offset: %.2fm", horizontal_offset);
            
            // Text rendering in C API requires initializing a font
            CvFont font;
            cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.6, 0.6, 0, 2, 8);
            cvPutText(small_frame, text_dist, cvPoint(10, 20), &font, cvScalar(0, 255, 0, 0));
            cvPutText(small_frame, text_off, cvPoint(10, 45), &font, cvScalar(0, 255, 0, 0));
            
            int target_x = (top_banner.cx + bot_banner.cx) / 2;
            int target_y = (top_banner.cy + bot_banner.cy) / 2;
            
            cvLine(small_frame, cvPoint(top_banner.cx, top_banner.cy), cvPoint(bot_banner.cx, bot_banner.cy), cvScalar(255, 0, 255, 0), 2, 8, 0);
            
            // C API lacks cv::drawMarker, so we manually draw a crosshair
            cvLine(small_frame, cvPoint(target_x - 10, target_y), cvPoint(target_x + 10, target_y), cvScalar(0, 255, 0, 0), 2, 8, 0);
            cvLine(small_frame, cvPoint(target_x, target_y - 10), cvPoint(target_x, target_y + 10), cvScalar(0, 255, 0, 0), 2, 8, 0);
        }
    }

    if (output_frame != NULL) {
        *output_frame = cv::cvarrToMat(small_frame, true);
    }

    if (show_window) {
        cvShowImage("3D Spatial Tracker (Pure C)", small_frame);
    }

    // CRITICAL MEMORY MANAGEMENT FOR PURE C
    // If you do not release these, your drone will run out of RAM and crash within seconds.
    cvReleaseImage(&small_frame);
    cvReleaseImage(&hsv);
    cvReleaseImage(&mask);
    cvReleaseImage(&temp);
    cvReleaseStructuringElement(&kernel);
    cvReleaseMemStorage(&storage);
}

int main(int argc, char** argv) {
    std::string folder_path = "Dataset_Images-20260319T151509Z-1-001/Dataset_Images";
    std::string output_video_path;
    if (argc > 1) {
        folder_path = argv[1];
    }
    if (argc > 2) {
        output_video_path = argv[2];
    }

    std::vector<fs::path> frame_files = get_sorted_frame_files(folder_path);
    if (frame_files.empty()) {
        printf("No image frames found in folder: %s\n", folder_path.c_str());
        printf("Usage: ./Gate_Tracker <frames_folder_path> [output_video.mp4]\n");
        cvDestroyAllWindows();
        return 1;
    }

    printf("Playing %zu frames from: %s\n", frame_files.size(), folder_path.c_str());
    if (!output_video_path.empty()) {
        printf("Exporting annotated video to: %s\n", output_video_path.c_str());
    } else {
        printf("Press 'q' or ESC to quit.\n");
    }

    cv::VideoWriter writer;
    bool enable_preview = output_video_path.empty();

    for (const auto& frame_path : frame_files) {
        cv::Mat img_mat = cv::imread(frame_path.string(), cv::IMREAD_COLOR);
        if (img_mat.empty()) {
            continue;
        }

        IplImage img = cvIplImage(img_mat);
        cv::Mat processed_frame;
        two_step_pnp_tracker_c(&img, enable_preview, &processed_frame);

        if (!output_video_path.empty() && !processed_frame.empty()) {
            if (!writer.isOpened()) {
                int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
                if (!writer.open(output_video_path, fourcc, 30.0, processed_frame.size(), true)) {
                    fprintf(stderr, "Failed to open video writer for output: %s\n", output_video_path.c_str());
                    cvDestroyAllWindows();
                    return 1;
                }
            }
            writer.write(processed_frame);
        }

        if (enable_preview) {
            int key = cvWaitKey(30);
            if (key == 27 || key == 'q' || key == 'Q') {
                break;
            }
        }
    }

    if (writer.isOpened()) {
        writer.release();
    }

    cvDestroyAllWindows();
    return 0;
}