//
// Created by leo on 18-11-13.
//

#include "Detecting.h"

namespace ORB_SLAM2
{
    Detecting::Detecting()
    {
        string ConfigPath = "/home/leo/Desktop/sematic_slam_project/src/sematic_slam/YOLO_V3/config/yolov3.cfg";
        string WeightsPath = "/home/leo/Desktop/Data/yolov3.weights";
        string MetaDataPath = "/home/leo/Desktop/sematic_slam_project/src/sematic_slam/YOLO_V3/config/coco.data";

        mpNetwork = load_network((char *) ConfigPath.data(), (char *) WeightsPath.data(), 0);
        mData = get_metadata((char *) MetaDataPath.data());
        mpTransMethod = new TransMethod;
    }

    void Detecting::SetTracker(ORB_SLAM2::Tracking *pTracker)
    {
        mpTracker = pTracker;
    }

    void Detecting::Run()
    {
        while (1)
        {
            if (CheckNewImage())//检查是否有新图片
            {
                PoccessNewImage();//获取新图片

                Detect();//对图片进行detect

                NotifyTracker();//唤醒等待Detecting线程的主线程
            }

            if (CheckFinish())
            {
                break;
            }
        }
    }

    void Detecting::RequRequestFinish()
    {
        unique_lock<mutex> lock(mMutexShutdown);//【锁】
        mbShutDownFlag = true;
    }

    //主线程调用，在GrabImageRGBD前面调用，将彩色图传至Detecting线程
    void Detecting::InsertImage(cv::Mat Image)
    {
        unique_lock<mutex> lock(mMutexImagesDeal);
        mlImages.push_back(Image);
    }

    //主线程调用，获取检测结果
    void Detecting::GetDetectResult(vector<DetectResult> &vspDetectResult)
    {
        unique_lock<mutex> lock(mMutexResultDeal);//【锁】
        vspDetectResult.assign(mvDetectResults.begin(),mvDetectResults.end());
    }

//____________________________________________以下是私有成员函数___________________________________________________

    bool Detecting::CheckFinish()
    {
        unique_lock<mutex> lock(mMutexShutdown);//【锁】
        return mbShutDownFlag;
    }

    bool Detecting::CheckNewImage()
    {
        unique_lock<mutex> lock(mMutexShutdown);//【锁】
        return (!mlImages.empty());
    }

    void Detecting::PoccessNewImage()
    {
        unique_lock<mutex> lock(mMutexImagesDeal);
        mCurrentImage = mlImages.front().clone();
        mlImages.pop_front();
    }

    void Detecting::NotifyTracker()
    {
        mpTracker->mconDetectFinished.notify_one();
    }

    void Detecting::Detect()
    {
        unique_lock<mutex> lock(mMutexResultDeal);//【锁】
        mvDetectResults.clear();
        image Image = mpTransMethod->MattoImage(mCurrentImage);//讲Mat数据转成Image类型

        //下面的检测过程是仿照python接口写的，还没有太弄明白是怎么回事,具体可能需要花时间看paper了，先把框架搭起来吧
        int *pCount = new int(0);
        network_predict_image(mpNetwork, Image);
        detection *pDetection = get_network_boxes(mpNetwork, Image.w, Image.h, 0.5, 0.5, nullptr, 0,
                                                  pCount);//第一步：get_network_boxes
        do_nms_obj(pDetection, *pCount, mData.classes, 0.45);//第二步：do_nms_obj

        //获取检测结果
        for (size_t j = 0; j < *pCount; j++)
        {
            for (size_t i = 0; i < mData.classes; i++)
            {
                if (pDetection[j].prob[i] > 0)
                {
                    DetectResult Result;
                    Result.mName = mData.names[i];
                    Result.mConfidence = pDetection[j].prob[i];
                    Result.mTop = (pDetection[j].bbox.y - pDetection[j].bbox.h / 2);
                    Result.mBottom = (pDetection[j].bbox.y + pDetection[j].bbox.h / 2);
                    Result.mLeft = (pDetection[j].bbox.x - pDetection[j].bbox.w / 2);
                    Result.mRight = (pDetection[j].bbox.x + pDetection[j].bbox.w / 2);
                    mvDetectResults.push_back(Result);
                }
            }
        }
    }

    void Detecting::Detect(cv::Mat Frame, vector<DetectResult>& vDetectResults)
    {
        vDetectResults.clear();
        image Image = mpTransMethod->MattoImage(Frame);//讲Mat数据转成Image类型

        //下面的检测过程是仿照python接口写的，还没有太弄明白是怎么回事,具体可能需要花时间看paper了，先把框架搭起来吧
        int *pCount = new int(0);
        network_predict_image(mpNetwork, Image);
        detection *pDetection = get_network_boxes(mpNetwork, Image.w, Image.h, 0.5, 0.5, nullptr, 0,
                                                  pCount);//第一步：get_network_boxes
        do_nms_obj(pDetection, *pCount, mData.classes, 0.45);//第二步：do_nms_obj

        //获取检测结果
        for (size_t j = 0; j < *pCount; j++)
        {
            for (size_t i = 0; i < mData.classes; i++)
            {
                if (pDetection[j].prob[i] > 0)
                {
                    DetectResult Result;
                    Result.mName = mData.names[i];
                    Result.mConfidence = pDetection[j].prob[i];
                    Result.mTop = (pDetection[j].bbox.y - pDetection[j].bbox.h / 2);
                    Result.mBottom = (pDetection[j].bbox.y + pDetection[j].bbox.h / 2);
                    Result.mLeft = (pDetection[j].bbox.x - pDetection[j].bbox.w / 2);
                    Result.mRight = (pDetection[j].bbox.x + pDetection[j].bbox.w / 2);
                    vDetectResults.push_back(Result);
                }
            }
        }

    }

    void Detecting::DrawResult( cv::Mat &Image, vector<DetectResult> Result)
    {
        for (vector<DetectResult>::iterator it = Result.begin(); it != Result.end(); it++)
        {
            cv::Point2f PointA(it->mLeft, it->mTop);
            cv::Point2f PointB(it->mRight, it->mBottom);
            cv::rectangle(Image, PointA, PointB, cv::Scalar(5, 100, 255), 5);
        }
    }
}