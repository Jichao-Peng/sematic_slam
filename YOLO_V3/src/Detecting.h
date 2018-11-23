//
// Created by leo on 18-11-13.
//

#ifndef PROJECT_DETECTING_H
#define PROJECT_DETECTING_H

#include <string>
#include <YOLO_V3/include/darknet.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <condition_variable>
#include "DetectResult.h"
#include "ORB_SLAM2/include/Tracking.h"


using namespace std;
namespace ORB_SLAM2
{
    class TransMethod;
    class Tracking;

    class Detecting
    {
    public:
        Detecting();

        void Run();

        void RequRequestFinish();

        void SetTracker(Tracking *pTracker);

        void InsertImage(cv::Mat Image);

        void PoccessNewImage();

        void Detect(cv::Mat Frame, vector<DetectResult>& vDetectResults);

        void GetDetectResult(vector<DetectResult>& vspDetectResult);

        void DrawResult( cv::Mat &Image, vector<DetectResult> Result);

    private:
        network *mpNetwork;
        metadata mData;

        TransMethod *mpTransMethod;
        Tracking *mpTracker;

        bool mbShutDownFlag = false;
        mutex mMutexShutdown;
        mutex mMutexImagesDeal;
        mutex mMutexResultDeal;
        cv::Mat mCurrentImage;

        list<cv::Mat> mlImages;
        vector<DetectResult> mvDetectResults;

        bool CheckFinish();

        bool CheckNewImage();

        void NotifyTracker();

        void Detect();
    };


    class TransMethod
    {
    public:
        image MattoImage(cv::Mat m)
        {
            IplImage ipl = m;
            image im = IpltoImage(&ipl);
            rgbgr_image(im);
            return im;
        }

    private:
        image IpltoImage(IplImage *src)
        {
            int h = src->height;
            int w = src->width;
            int c = src->nChannels;
            image im = make_image(w, h, c);
            unsigned char *data = (unsigned char *) src->imageData;
            int step = src->widthStep;
            int i, j, k;

            for (i = 0; i < h; ++i)
            {
                for (k = 0; k < c; ++k)
                {
                    for (j = 0; j < w; ++j)
                    {
                        im.data[k * w * h + i * w + j] = data[i * step + j * c + k] / 255.;
                    }
                }
            }
            return im;
        }
    };
}
#endif //PROJECT_DETECTING_H
