#include <opencv/cxcore.h>
#include <opencv/cvaux.h>
#include "lane_data.h"
#include <stdexcept>
#include "../../eklavya2.h"
#define choice 2
#define scale 220/57
#define N 7 // canny kernel 

#define SCALE_X 10

#define MAP_MAX 1000
int mouseParam = 5;

sensor_msgs::CvBridge bridge;
static int iter = 0;
IplImage *kernel_frame;
IplImage *edge_frame;
IplImage *gray_hough_frame;
IplImage *gray_frame;
IplImage *lane;
IplImage *warp_img;
IplImage* img;
int i;
LaneDetection lane_o;
CvPoint offset;
CvPoint2D32f srcQuad[4], dstQuad[4];
CvMat* warp_matrix = cvCreateMat(3, 3, CV_32FC1);

int canny_kernel = 7, high_threshold = 500, low_threshold = 300, vote = 25, length = 25, mrg = 5, maxvalue_R = 255, maxvalue_G = 255, maxvalue_B = 255, minvalue_R = 210, minvalue_G = 210, minvalue_B = 210;

IplImage* LaneDetection::colorBasedLaneDetection(IplImage *frame, int maxvalue_B, int maxvalue_G, int maxvalue_R, int minvalue_B, int minvalue_G, int minvalue_R) {
    uchar* frame_data;
    uchar* frame_out_data;
    int i, j, index, index_out;
    IplImage *frame_out = cvCreateImage(cvGetSize(frame), frame->depth, 1);
    frame_out_data = (uchar*) frame_out->imageData;
    frame_data = (uchar*) frame->imageData;
    for (i = 0; i < frame->height; i++) {
        for (j = 0; j < frame->width; j++) {
            index = (i * frame->widthStep)+(j * frame->nChannels);
            index_out = (i * frame_out->widthStep) + j;
            if ((frame_data[index + 0] <= maxvalue_B) && (frame_data[index + 0] >= minvalue_B) &&
                    (frame_data[index + 1] <= maxvalue_G) && (frame_data[index + 1] >= minvalue_G) &&
                    (frame_data[index + 2] <= maxvalue_R) && (frame_data[index + 2] >= minvalue_R)
                    ) {
                frame_out_data[index_out] = 255;
            } else {
                frame_out_data[index_out] = 0;
            }
        }
    }
    cvDilate(frame_out, frame_out, 0, 2);
    return frame_out;
}

void LaneDetection::applyHoughTransform(IplImage* img, IplImage *dst, int vote, int length, int mrgh) {
    CvSeq* lines;
    CvMemStorage* storage = cvCreateMemStorage(0);
    cvSetZero(dst);
    int i;
    lines = cvHoughLines2(img, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI / 180, vote, length, mrgh);
    int n = lines->total;
    for (i = 0; i < n; i++) {
        CvPoint* line = (CvPoint*) cvGetSeqElem(lines, i);
        cvLine(dst, line[0], line[1], CV_RGB(255, 255, 255), 15, 8);
    }
    cvReleaseMemStorage(&storage);
}

CvSeq* GetHoughLanes(IplImage* img, int vote, int length, int mrgha) {
    CvSeq* lines;
    CvMemStorage* storage = cvCreateMemStorage(0);
    lines = cvHoughLines2(img, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI / 180, vote, length, mrgha);

    return lines;
}

void LaneDetection::initializeLaneVariables(IplImage *input_frame) {
    offset = cvPoint((N - 1) / 2, (N - 1) / 2);
    gray_frame = cvCreateImage(cvGetSize(input_frame), input_frame->depth, 1);
    kernel_frame = cvCreateImage(cvSize(gray_frame->width + N - 1, gray_frame->height + N - 1), gray_frame->depth, gray_frame->nChannels);
    edge_frame = cvCreateImage(cvGetSize(kernel_frame), IPL_DEPTH_8U, kernel_frame->nChannels);
    gray_hough_frame = cvCreateImage(cvGetSize(kernel_frame), IPL_DEPTH_8U, kernel_frame->nChannels);
    cvNamedWindow("Control Box", 1);
    cvCreateTrackbar("B Max", "Control Box", &maxvalue_B, 255, NULL);
    cvCreateTrackbar("G Max", "Control Box", &maxvalue_G, 255, NULL);
    cvCreateTrackbar("R Max", "Control Box", &maxvalue_R, 255, NULL);
    cvCreateTrackbar("B Min", "Control Box", &minvalue_B, 255, NULL);
    cvCreateTrackbar("G Min", "Control Box", &minvalue_G, 255, NULL);
    cvCreateTrackbar("R Min", "Control Box", &minvalue_R, 255, NULL);
    cvCreateTrackbar("Vote", "Control Box", &vote, 50, NULL);
    cvCreateTrackbar("Length", "Control Box", &length, 100, NULL);
    cvCreateTrackbar("Merge", "Control Box", &mrg, 15, NULL);
    cvCreateTrackbar("Canny Kernel", "Control Box", &canny_kernel, 10, NULL);
    cvCreateTrackbar("High Canny Threshold", "Control Box", &high_threshold, 1000, NULL);
    cvCreateTrackbar("Low Canny Threshold", "Control Box", &low_threshold, 500, NULL);

    //Destination variables
    int widthInCM = 275, h1 = 320, h2 = 880; //width and height of the lane. width:widthoflane/scale;

    srcQuad[0].x = (float) 212; //src Top left

    srcQuad[0].y = (float) 266;
    srcQuad[1].x = (float) 413; //src Top right
    srcQuad[1].y = (float) 282;
    srcQuad[2].x = (float) 23; //src Bottom left
    srcQuad[2].y = (float) 406;
    srcQuad[3].x = (float) 581; //src Bot right
    srcQuad[3].y = (float) 445;

    dstQuad[0].x = (float) (500 - widthInCM / (2)); //dst Top left
    dstQuad[0].y = (float) (999 - h2);
    dstQuad[1].x = (float) (500 + widthInCM / (2)); //dst Top right
    dstQuad[1].y = (float) (999 - h2);
    dstQuad[2].x = (float) (500 - widthInCM / (2)); //dst Bottom left
    dstQuad[2].y = (float) (999 - h1);
    dstQuad[3].x = (float) (500 + widthInCM / (2)); //dst Bot right
    dstQuad[3].y = (float) (999 - h1);

    //    printf("Hello ");
    cvGetPerspectiveTransform(dstQuad, srcQuad, warp_matrix);
    //    printf("World\n");

}

IplImage *getLaneLines(IplImage* src) {
    IplImage* dst = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
    IplImage* color_dst = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 3);
    CvMemStorage* storage = cvCreateMemStorage(0);
    CvSeq* lines = 0;
    int i;
    cvCanny(src, dst, 50, 200, 3);

    color_dst = cvCreateImage(cvGetSize(dst), dst->depth, 3);
    cvSetZero(color_dst);
    //cvCvtColor( dst, color_dst, CV_GRAY2BGR );

    lines = cvHoughLines2(dst, storage, CV_HOUGH_STANDARD, 1, CV_PI / 180, 60, 0, 0);

    for (i = 0; i < MIN(lines->total, 100); i++) {
        float* line = (float*) cvGetSeqElem(lines, i);
        float rho = line[0];
        float theta = line[1];
        CvPoint pt1, pt2;
        //if( !((10*theta>=24 && 10*theta<=30)||(100*theta>=50 && 100*theta<=70)) )
        //{
        //printf("%.2f \n",theta);

        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000 * (-b));
        pt1.y = cvRound(y0 + 1000 * (a));
        pt2.x = cvRound(x0 - 1000 * (-b));
        pt2.y = cvRound(y0 - 1000 * (a));
        cvLine(color_dst, pt1, pt2, CV_RGB(255, 255, 255), 3, 8);
        //}
    }

    cvCvtColor(color_dst, dst, CV_BGR2GRAY);
    return dst;
}

void markPixel(IplImage *img, int r, int c) {
    int i, j;
    uchar *data = (uchar*) img->imageData;
    for (i = -2; i < 0; i++) {
        for (j = -2; j < 2; j++) {
            data[(r + i) * img->widthStep + (c + j) * img->nChannels + 0] = 255;
            data[(r + i) * img->widthStep + (c + j) * img->nChannels + 1] = 255;
            data[(r + i) * img->widthStep + (c + j) * img->nChannels + 2] = 255;
        }
    }
}

int min(int value) {
    if (value > 999)
        return 999;
    if (value < 0)
        return 0;
}

void populateLanes(IplImage *img) {
    int i, j, index, k;
    pthread_mutex_lock(&laser_scan_mutex);
    for (i = 0; i < img->height; i++) {
        uchar *data = (uchar *) (img->imageData + i * img->widthStep);
        for (j = 0; j < img->width; j++) {
            for (k = 0; k < scale; k++)
                cam_input[(int) (min((0.5 * MAP_MAX) + (scale * (j - img->width / 2)) + k)) ] [(int) (img->height - (i) + (0.1 * MAP_MAX)) ] = (data[j] == 255) ? 255 : 0;
        }
    }
    pthread_mutex_unlock(&laser_scan_mutex);

}

void mouseHandler(int event, int x, int y, int flags, void* param) {

    if (event == CV_EVENT_RBUTTONDOWN) {

        i++;

        std::cout << "@ Right mouse button pressed at: " << x << "," << y << std::endl;
    }
}

void LaneDetection::markLane(const sensor_msgs::ImageConstPtr& image) {
    //lane =  LaneDetection::colorBasedLaneDetection(input_frame,maxvalue_B,maxvalue_G,maxvalue_R,minvalue_B,minvalue_G,minvalue_R,vote,length,mrg);


    try {
        img = bridge.imgMsgToCv(image, "bgr8");
        cvWaitKey(10);
    } catch (sensor_msgs::CvBridgeException& e) {
        ROS_ERROR("ERROR IN COVERTING IMAGE!!!");
    }


    cvShowImage("view_orig", img);
    cvWaitKey(10);

    if (iter == 0)
        initializeLaneVariables(img);
    iter++;
    if (choice == 1) {
        lane = lane_o.colorBasedLaneDetection(img, maxvalue_B, maxvalue_G, maxvalue_R, minvalue_B, minvalue_G, minvalue_R);
    }
    if ((choice == 2) || (choice == 0)) {
        cvCvtColor(img, gray_frame, CV_BGR2GRAY); // converting image to gray scale

        cvEqualizeHist(gray_frame, gray_frame);

        // canny edge detection
        cvCopyMakeBorder(gray_frame, kernel_frame, offset, IPL_BORDER_REPLICATE, cvScalarAll(0));
        cvCanny(kernel_frame, edge_frame, low_threshold * canny_kernel*canny_kernel, high_threshold * canny_kernel*canny_kernel, canny_kernel);
        lane_o.applyHoughTransform(edge_frame, gray_hough_frame, vote, length, mrg);
    }
    if (choice == 0) {
        lane = gray_hough_frame;
    }
    if (choice == 2) {
        lane = joinResult(lane_o.colorBasedLaneDetection(img, maxvalue_B, maxvalue_G, maxvalue_R, minvalue_B, minvalue_G, minvalue_R), gray_hough_frame);
    }

    cvSetMouseCallback("view", &mouseHandler, 0);
    cvShowImage("view", lane);
    cvWaitKey(10);

    warp_img = cvCreateImage(cvSize(MAP_MAX, MAP_MAX), img->depth, 1);

    cvWarpPerspective(lane, warp_img, warp_matrix, CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS);

    cvShowImage("warp", warp_img);
    cvWaitKey(10);

    //May or may not be required depends upon the results.
    //warp_img = getLaneLines(warp_img);
    //cvDilate(warp_img,warp_img,0,10);
    //getDestinationPosition(warp_img);

    pthread_mutex_lock(&laser_scan_mutex);
    pthread_mutex_unlock(&laser_scan_mutex);
    //populateLanes(warp_img);

    //Release warp_img and input frame and lane.
    cvReleaseImage(&warp_img);
    //    cvReleaseImage(&img);
    cvReleaseImage(&lane);

}

IplImage* LaneDetection::joinResult(IplImage* color_gray, IplImage* hough_gray) {
    int i, j;
    int index_color;
    int index_hough;
    IplImage* lane_gray = cvCreateImage(cvGetSize(color_gray), color_gray->depth, 1);

    uchar* color_data = (uchar*) color_gray->imageData;
    uchar* hough_data = (uchar*) hough_gray->imageData;
    uchar* lane_data = (uchar*) lane_gray->imageData;

    for (i = 0; i < color_gray->height; i++) {
        for (j = 0; j < color_gray->width; j++) {
            index_color = (i * color_gray->widthStep)+(j);
            index_hough = ((i + ((canny_kernel - 1))) * hough_gray->widthStep)+((j + ((canny_kernel - 1))));

            if ((color_data[index_color] > 0) && (hough_data[index_hough] > 0))
                lane_data[index_color] = 255;
        }
    }
    return lane_gray;
}

