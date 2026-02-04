#include "GxIAPI.h"
#include <iostream>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#ifndef CAMERA.HPP
#define CAMERA.HPP
// Abstract class, with callback function type as a parameter
// template<typename CallBackType>
class AbstractCamera {

    public:
        
        int width;
        int height;
        int offsetX = 0;
        int offsetY = 0;

       // virtual void setCallBack(CallBackType cllbck) = 0;

        virtual void setWidth(int w) = 0;
        
        virtual void setHeight(int h) = 0;
        
        virtual void setOffsetX(int x) = 0;
        
        virtual void setOffsetY(int y) = 0;

        virtual void startCapture() = 0;

        virtual void stopCapture() = 0;


};

class AICamera : AbstractCamera {

    public:

        GX_DEV_HANDLE hDevice;

        AICamera();

        void init();

       // void setCallBack(GXCaptureCallBack cllbck) override;

        void setWidth(int w) override;

        void setHeight(int h) override;

        void setOffsetX(int x) override;
        
        void setOffsetY(int y) override;

        void startCapture() override;

        void stopCapture() override;

        void startStream();

        void stopStream();

        int getBuffer(PGX_FRAME_BUFFER* pFrameBuffer);

        int returnBuffer(PGX_FRAME_BUFFER* pFrameBuffer);

        void destroy();
};

class V4L2Camera : AbstractCamera {

    public:

        V4L2Camera(std::string path);

        V4L2Camera();

        struct v4l2_format fmt;
        int fd;
        int buffers[6];

        int init(std::string path);

        void setWidth(int w) override;

        void setHeight(int h) override;

        void setOffsetX(int x) override;
        
        void setOffsetY(int y) override;

        void startCapture() override;

        void stopCapture() override;

        void getBuffer(struct v4l2_buffer** buf);

        void destroy();


};

#endif