#include "../include/camera.hpp"

AICamera::AICamera() {}
// void AICamera::setCallBack(GXCaptureCallBack cllbck) {
//     GXRegisterCaptureCallback(hDevice, NULL, cllbck);
// }

void AICamera::init() {

    int status = GXInitLib();

    if (status != GX_STATUS_SUCCESS) {
        std::cout << "Library initialization failed : ";
        std::cout << status << std::endl;
        return;
    }

    //Updates the enumeration list for the devices.
    uint32_t nDeviceNum = 0;
    status = GXUpdateDeviceList(&nDeviceNum, 1000);
    if ((status != GX_STATUS_SUCCESS) || (nDeviceNum <= 0)) {
        std::cout << "Update list failed" << std::endl;
        return;
    }
    //Opens the device.
    status = GXOpenDeviceByIndex(1, &hDevice);

    if (status != GX_STATUS_SUCCESS) {
        std::cout << "Failed opening" << std::endl;
        return;
    }

    GXSetAcqusitionBufferNumber(hDevice, 25);
}

void AICamera::setWidth(int w) {
    width = w;
    GXSetInt(hDevice, GX_INT_WIDTH, w);
}

void AICamera::setHeight(int h) {
    height = h;
    GXSetInt(hDevice, GX_INT_HEIGHT, h);
}

void AICamera::setOffsetX(int x) {
    offsetX = x;
    GXSetInt(hDevice, GX_INT_OFFSET_X, x);
}

void AICamera::setOffsetY(int y) {
    offsetY = y;
    GXSetInt(hDevice, GX_INT_OFFSET_Y, y);
}

void AICamera::startCapture() {
    GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_START);
}

void AICamera::startStream() {
    GXStreamOn(hDevice);
}

void AICamera::stopStream() {
    GXStreamOff(hDevice);
}
int AICamera::getBuffer(PGX_FRAME_BUFFER* pFrameBuffer) {
    return GXDQBuf(hDevice, pFrameBuffer, 1000);
}
int AICamera::returnBuffer(PGX_FRAME_BUFFER* pFrameBuffer) {
    return GXQBuf(hDevice, *pFrameBuffer);
}
void AICamera::stopCapture() {
    GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_STOP);

    GXUnregisterCaptureCallback(hDevice);
    GXCloseDevice(hDevice);
    GXCloseLib();
}

void AICamera::destroy() {
    GXCloseDevice(hDevice);
    GXCloseLib();
}


V4L2Camera::V4L2Camera(std::string path) {

    fd = open(path.c_str(), O_RDWR);
    fmt = {0};


}

V4L2Camera::V4L2Camera() {}

void V4L2Camera::setWidth(int w) {

    fmt.fmt.pix.width = w;

}

void V4L2Camera::setHeight(int h) {

    fmt.fmt.pix.height = h;

}

int V4L2Camera::init(std::string path) {

    fd = open(path.c_str(), O_RDWR);
    fmt = {0};

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("VIDIOC_S_FMT");
        return 1;
    }

    struct v4l2_requestbuffers req = {0};
    req.count = 10;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("VIDIOC_REQBUFS");
        return 1;
    }

    

    for (int i = 0; i < req.count; i++) {

        struct v4l2_buffer buf = {};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("VIDIOC_QUERYBUF");
            return 1;
        }

        struct v4l2_exportbuffer exp_buf = {0};

        exp_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        exp_buf.index = i;
        exp_buf.plane = 0;
        exp_buf.flags = O_CLOEXEC;


        if (ioctl(fd, VIDIOC_EXPBUF, &exp_buf) < 0) {
            perror("VIDIOC_EXPBUF");
            return 1;
        }

        buffers[i] = exp_buf.fd;
        sizes[i] = buf.length;
        if(ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            perror("VIDIOC_QBUF");
            return 1;
        }

    }
}

void V4L2Camera::startCapture() {

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        perror("VIDIOC_STREAMON");
    }

}

void V4L2Camera::setOffsetX(int x) {}

void V4L2Camera::setOffsetY(int y) {}

void V4L2Camera::stopCapture() {

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
        perror("VIDIOC_STREAMOFF");
    }

}

void V4L2Camera::getBuffer(struct v4l2_buffer** buf) {

    if (ioctl(fd, VIDIOC_DQBUF, *buf) < 0) {
        perror("VIDIOC_DQBUF");
    }
}

void V4L2Camera::destroy() {
    close(fd);
}