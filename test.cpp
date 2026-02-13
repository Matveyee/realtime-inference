// minimal_v4l2_mpp_mjpeg.cpp

#include <iostream>
#include <vector>
#include <cstring>
#include <cstdlib>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <linux/videodev2.h>

// Заголовки MPP (пути могут отличаться в вашей системе)
extern "C" {
#include "include/inc/rk_mpi.h"
#include "include/inc/mpp_frame.h"
#include "include/inc/mpp_packet.h"
#include "include/inc/mpp_log.h"
}

#define CAM_DEV        "/dev/video40"
#define CAM_WIDTH      1920
#define CAM_HEIGHT     1080
#define V4L2_BUFFERS   4
#define MAX_FRAME_SIZE (4 * 1024 * 1024) // максимум под один MJPEG кадр

struct V4L2Buffer {
    void   *start = nullptr;
    size_t  length = 0;
};

int main() {
    int ret = 0;

    // ========= 1. Открываем камеру =========
    int cam_fd = open(CAM_DEV, O_RDWR | O_NONBLOCK);
    if (cam_fd < 0) {
        perror("open camera");
        return 1;
    }

    // (не обязательно, но полезно) проверим возможности
    struct v4l2_capability cap = {};
    if (ioctl(cam_fd, VIDIOC_QUERYCAP, &cap) < 0) {
        perror("VIDIOC_QUERYCAP");
        close(cam_fd);
        return 1;
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        std::cerr << "Device is not a capture device\n";
        close(cam_fd);
        return 1;
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        std::cerr << "Device does not support streaming I/O\n";
        close(cam_fd);
        return 1;
    }

    // ========= 2. Настраиваем формат (MJPEG) =========
    struct v4l2_format fmt = {};
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = CAM_WIDTH;
    fmt.fmt.pix.height      = CAM_HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field       = V4L2_FIELD_ANY;

    if (ioctl(cam_fd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("VIDIOC_S_FMT");
        close(cam_fd);
        return 1;
    }

    std::cout << "Camera format set: "
              << fmt.fmt.pix.width << "x" << fmt.fmt.pix.height
              << " fourcc=" << std::hex << fmt.fmt.pix.pixelformat << std::dec
              << std::endl;

    // ========= 3. Запрашиваем буферы MMAP =========
    struct v4l2_requestbuffers req = {};
    req.count  = V4L2_BUFFERS;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(cam_fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("VIDIOC_REQBUFS");
        close(cam_fd);
        return 1;
    }

    if (req.count < V4L2_BUFFERS) {
        std::cerr << "Not enough V4L2 buffers\n";
        close(cam_fd);
        return 1;
    }

    std::vector<V4L2Buffer> buffers(req.count);

    for (unsigned int i = 0; i < req.count; ++i) {
        struct v4l2_buffer buf = {};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;

        if (ioctl(cam_fd, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("VIDIOC_QUERYBUF");
            close(cam_fd);
            return 1;
        }

        buffers[i].length = buf.length;
        buffers[i].start = mmap(nullptr, buf.length,
                                PROT_READ | PROT_WRITE,
                                MAP_SHARED,
                                cam_fd,
                                buf.m.offset);
        if (buffers[i].start == MAP_FAILED) {
            perror("mmap");
            close(cam_fd);
            return 1;
        }

        // Ставим буфер в очередь
        if (ioctl(cam_fd, VIDIOC_QBUF, &buf) < 0) {
            perror("VIDIOC_QBUF");
            close(cam_fd);
            return 1;
        }
    }

    // ========= 4. Запускаем стрим камеры =========
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(cam_fd, VIDIOC_STREAMON, &type) < 0) {
        perror("VIDIOC_STREAMON");
        close(cam_fd);
        return 1;
    }

    // ========= 5. Инициализируем MPP (MJPEG декодер) =========
    MppCtx ctx  = nullptr;
    MppApi *mpi = nullptr;

    ret = mpp_create(&ctx, &mpi);
    if (ret) {
        std::cerr << "mpp_create failed: " << ret << std::endl;
        return 1;
    }

    // Можно включить лог MPP покруче для отладки
    // mpp_log_set_flag(0xFFFFFFFF);

    ret = mpp_init(ctx, MPP_CTX_DEC, MPP_VIDEO_CodingMJPEG);
    if (ret) {
        std::cerr << "mpp_init MJPEG failed: " << ret << std::endl;
        return 1;
    }

    // Опционально — игнорировать ошибки потока, чтобы декодер пытался выводить кадры
    {
        RK_U32 disable_err = 1;
        mpi->control(ctx, MPP_DEC_SET_DISABLE_ERROR, &disable_err);
    }

    std::cout << "MPP MJPEG decoder inited\n";

    // Буфер для копии JPEG (чтобы не париться с lifetime MPP)
    std::vector<uint8_t> jpeg_buf(MAX_FRAME_SIZE);

    int frame_count = 0;

    // ========= 6. Главный цикл захвата + декодирования =========
    while (frame_count < 100) { // для примера остановимся на 100 кадрах
        // 6.1 Ждём кадр: DQBUF
        struct v4l2_buffer v4l2buf = {};
        v4l2buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2buf.memory = V4L2_MEMORY_MMAP;

        // Блокирующий ожидание удобнее: если NONBLOCK — можно через select/poll
        if (ioctl(cam_fd, VIDIOC_DQBUF, &v4l2buf) < 0) {
            perror("VIDIOC_DQBUF");
            usleep(1000);
            continue;
        }

        int index = v4l2buf.index;
        size_t used = v4l2buf.bytesused;
        if (used == 0 || used > MAX_FRAME_SIZE) {
            std::cerr << "Bad frame size: " << used << std::endl;
            // Вернём буфер и дальше
            ioctl(cam_fd, VIDIOC_QBUF, &v4l2buf);
            continue;
        }

        std::cout << "[V4L2] got buffer index=" << index
                  << " bytesused=" << used << std::endl;

        // 6.2 Копируем MJPEG кадр в обычный буфер
        std::memcpy(jpeg_buf.data(), buffers[index].start, used);

        // 6.3 Готовим MppPacket поверх этого буфера
        MppPacket packet = nullptr;
        ret = mpp_packet_init(&packet, jpeg_buf.data(), MAX_FRAME_SIZE);
        if (ret) {
            std::cerr << "mpp_packet_init failed: " << ret << std::endl;
            ioctl(cam_fd, VIDIOC_QBUF, &v4l2buf);
            continue;
        }
        mpp_packet_set_length(packet, used);
        mpp_packet_set_pos(packet, jpeg_buf.data());

        // 6.4 Кладём пакет в декодер
        ret = mpi->decode_put_packet(ctx, packet);
        if (ret && ret != MPP_ERR_BUFFER_FULL) {
            std::cerr << "decode_put_packet failed: " << ret << std::endl;
            mpp_packet_deinit(&packet);
            ioctl(cam_fd, VIDIOC_QBUF, &v4l2buf);
            break;
        }
        mpp_packet_deinit(&packet);

        // 6.5 Возвращаем буфер камере
        if (ioctl(cam_fd, VIDIOC_QBUF, &v4l2buf) < 0) {
            perror("VIDIOC_QBUF");
            break;
        }

        // 6.6 Пытаемся вытащить все готовые декодированные кадры
        while (true) {
            MppFrame frame = nullptr;
            ret = mpi->decode_get_frame(ctx, &frame);
            if (ret != MPP_OK) {
                std::cerr << "decode_get_frame ret=" << ret << std::endl;
                break;
            }

            if (!frame) {
                // на сейчас готовых кадров нет
                break;
            }

            if (mpp_frame_get_info_change(frame)) {
                int w  = mpp_frame_get_width(frame);
                int h  = mpp_frame_get_height(frame);
                int hs = mpp_frame_get_hor_stride(frame);
                int vs = mpp_frame_get_ver_stride(frame);
                size_t buf_size = mpp_frame_get_buf_size(frame);

                std::cout << "[MPP] INFO_CHANGE: " << w << "x" << h
                          << " stride " << hs << "x" << vs
                          << " buf_size " << buf_size << std::endl;

                // Ничего внешнего не настраиваем — используем внутренние буферы MPP
                mpi->control(ctx, MPP_DEC_SET_INFO_CHANGE_READY, nullptr);
                mpp_frame_deinit(&frame);
                continue;
            }

            int err = mpp_frame_get_errinfo(frame);
            int dsc = mpp_frame_get_discard(frame);
            if (err || dsc) {
                std::cout << "[MPP] frame err=" << err
                          << " discard=" << dsc << std::endl;
                mpp_frame_deinit(&frame);
                continue;
            }

            int out_w = mpp_frame_get_width(frame);
            int out_h = mpp_frame_get_height(frame);
            std::cout << "[MPP] GOT DECODED FRAME #" << frame_count
                      << " " << out_w << "x" << out_h << std::endl;

            // Здесь можно получить MppBuffer с NV12 и что-то с ним сделать
            // MppBuffer frame_buf = mpp_frame_get_buffer(frame);
            // void *ptr = mpp_buffer_get_ptr(frame_buf);
            // size_t frame_size = mpp_frame_get_buf_size(frame);

            mpp_frame_deinit(&frame);
            frame_count++;
        }
    }

    std::cout << "Decoded " << frame_count << " frames\n";

    // ========= 7. Остановка и очистка =========
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(cam_fd, VIDIOC_STREAMOFF, &type);

    for (auto &b : buffers) {
        if (b.start && b.length)
            munmap(b.start, b.length);
    }

    close(cam_fd);

    if (ctx && mpi) {
        mpi->reset(ctx);
        mpp_destroy(ctx);
    }

    return 0;
}
