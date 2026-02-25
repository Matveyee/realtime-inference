#include <iostream>
#include <vector>
#include <cstring>
#include <cerrno>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>

#include <linux/videodev2.h>

extern "C" {
#include "include/inc/rk_mpi.h"
#include "include/inc/mpp_packet.h"
#include "include/inc/mpp_frame.h"
// #include "mpp_dec_cfg.h"
}

static void die(const char* msg) {
    perror(msg);
    std::exit(1);
}

struct MMapBuf {
    void*  ptr = nullptr;
    size_t len = 0;
};




static MppBufferGroup out_grp = nullptr;

static void on_info_change(MppCtx ctx, MppApi* mpi, MppFrame frame) {
    RK_U32 w  = mpp_frame_get_width(frame);
    RK_U32 h  = mpp_frame_get_height(frame);
    RK_U32 hs = mpp_frame_get_hor_stride(frame);
    RK_U32 vs = mpp_frame_get_ver_stride(frame);
    RK_U32 buf_size = mpp_frame_get_buf_size(frame);

    std::cerr << "[INFO_CHANGE] " << w << "x" << h
              << " stride " << hs << "x" << vs
              << " buf_size=" << buf_size << "\n";

    if (!out_grp) {
        // ВНУТРЕННЯЯ группа (самый простой режим)
        // Тип может быть ION/DRM в зависимости от сборки MPP.
        // На многих RK работает MPP_BUFFER_TYPE_ION.
        MPP_RET ret = mpp_buffer_group_get_internal(&out_grp, MPP_BUFFER_TYPE_ION);
        if (ret) {
            std::cerr << "mpp_buffer_group_get_internal ret=" << ret << "\n";
            return;
        }

        // Ограничиваем пул: например, 24 кадра по buf_size
        ret = mpp_buffer_group_limit_config(out_grp, buf_size, 24);
        if (ret) {
            std::cerr << "mpp_buffer_group_limit_config ret=" << ret << "\n";
            return;
        }

        ret = mpi->control(ctx, MPP_DEC_SET_EXT_BUF_GROUP, out_grp);
        if (ret) {
            std::cerr << "MPP_DEC_SET_EXT_BUF_GROUP ret=" << ret << "\n";
            return;
        }
    }

    // Разрешаем декодеру продолжать
    MPP_RET ret = mpi->control(ctx, MPP_DEC_SET_INFO_CHANGE_READY, nullptr);
    if (ret) {
        std::cerr << "MPP_DEC_SET_INFO_CHANGE_READY ret=" << ret << "\n";
    }
}

int put_with_retry(MppCtx ctx, MppApi* mpi, MppPacket pkt) {
    for (;;) {
        MPP_RET ret = mpi->decode_put_packet(ctx, pkt);
        std::cerr << "decode_put_packet ret=" << ret << "\n";
        if (ret == MPP_OK) return 0;

        if (ret != MPP_ERR_BUFFER_FULL) {
            std::cerr << "decode_put_packet ret=" << ret << "\n";
            return -1;
        }

        // очередь полна — выкачиваем кадры
        while (true) {
            MppFrame frame = nullptr;
            MPP_RET gr = mpi->decode_get_frame(ctx, &frame);
            std::cerr << "decode_get_frame ret=" << ret << "\n";
            if (ret == MPP_ERR_BUFFER_FULL) {
                return -1;
            }
            if (gr != MPP_OK) break;
            if (!frame) break;

            if (mpp_frame_get_info_change(frame)) {
                // обработаем ниже (см. пункт 2)
                // ...
                on_info_change(ctx, mpi, frame);
            }
            mpp_frame_deinit(&frame);
        }

        usleep(1000);
    }
}

int main() {
    const char* dev = "/dev/video40";
    const int   req_w = 1280;
    const int   req_h = 720;
    const int   buf_count = 4;

    // 1) Open camera (BLOCKING проще для старта)
    int fd = open(dev, O_RDWR);
    if (fd < 0) die("open");

    // 2) Set format: MJPEG
    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = req_w;
    fmt.fmt.pix.height = req_h;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) die("VIDIOC_S_FMT");

    std::cout << "V4L2 format: " << fmt.fmt.pix.width << "x" << fmt.fmt.pix.height
              << " fourcc=0x" << std::hex << fmt.fmt.pix.pixelformat << std::dec << "\n";

    // 3) Request MMAP buffers
    v4l2_requestbuffers req{};
    req.count = buf_count;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) die("VIDIOC_REQBUFS");
    if (req.count < 2) {
        std::cerr << "Not enough V4L2 buffers\n";
        return 1;
    }

    std::vector<MMapBuf> bufs(req.count);

    for (unsigned i = 0; i < req.count; ++i) {
        v4l2_buffer b{};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = i;

        if (ioctl(fd, VIDIOC_QUERYBUF, &b) < 0) die("VIDIOC_QUERYBUF");

        bufs[i].len = b.length;
        bufs[i].ptr = mmap(nullptr, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        if (bufs[i].ptr == MAP_FAILED) die("mmap");

        if (ioctl(fd, VIDIOC_QBUF, &b) < 0) die("VIDIOC_QBUF");
    }

    // 4) STREAMON
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) die("VIDIOC_STREAMON");

    // 5) Init MPP decoder (MJPEG)
    MppCtx ctx = nullptr;
    MppApi* mpi = nullptr;

    if (mpp_create(&ctx, &mpi) != MPP_OK) {
        std::cerr << "mpp_create failed\n";
        return 1;
    }
    if (mpp_init(ctx, MPP_CTX_DEC, MPP_VIDEO_CodingMJPEG) != MPP_OK) {
        std::cerr << "mpp_init MJPEG failed\n";
        return 1;
    }

    // // В стиле mpi_dec_test: включим split_parse (на всякий случай)
    // {
    //     MppDecCfg cfg = nullptr;
    //     mpp_dec_cfg_init(&cfg);
    //     mpi->control(ctx, MPP_DEC_GET_CFG, cfg);
    //     mpp_dec_cfg_set_u32(cfg, "base:split_parse", 1);
    //     mpi->control(ctx, MPP_DEC_SET_CFG, cfg);
    //     mpp_dec_cfg_deinit(cfg);
    // }

    // Optional: не отбрасывать кадры из-за ошибок потока на старте отладки
    {
        RK_U32 disable_err = 1;
        mpi->control(ctx, MPP_DEC_SET_DISABLE_ERROR, &disable_err);
    }

    // Packet (как в mpi_dec_test simple): один раз init, потом переиспользуем
    MppPacket pkt = nullptr;
    if (mpp_packet_init(&pkt, nullptr, 0) != MPP_OK) {
        std::cerr << "mpp_packet_init failed\n";
        return 1;
    }

    int decoded = 0;

    // 6) Main loop
    while (decoded < 100) {
        // Ждём готовый буфер через select (стабильно и без EAGAIN-спама)
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
        timeval tv{2, 0};
        v4l2_buffer b{};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;

        std::cerr << "before DQBUF" << std::endl;
        if (ioctl(fd, VIDIOC_DQBUF, &b) < 0) die("VIDIOC_DQBUF");
        std::cerr << "after DQBUF bytesused=" << b.bytesused << " index=" << b.index << std::endl;



        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd, VIDIOC_DQBUF, &b) < 0) die("VIDIOC_DQBUF");

        const int idx = b.index;
        const size_t used = b.bytesused;

        if (used == 0 || used > bufs[idx].len) {
            std::cerr << "Bad bytesused=" << used << " buf_len=" << bufs[idx].len << "\n";
            if (ioctl(fd, VIDIOC_QBUF, &b) < 0) die("VIDIOC_QBUF");
            continue;
        }

        // 6.1) Feed MPP with MJPEG bytes from MMAP buffer
        void* data = bufs[idx].ptr;
        mpp_packet_set_data(pkt, data);
        mpp_packet_set_pos(pkt, data);
        mpp_packet_set_size(pkt, bufs[idx].len);   // capacity
        mpp_packet_set_length(pkt, used);          // actual bytes

        int r = put_with_retry(ctx, mpi, pkt);
        std::cerr << "put_with_retry ret=" << r << std::endl;
        if (r && r != MPP_ERR_BUFFER_FULL) {
            std::cerr << "decode_put_packet ret=" << r << "\n";
        }

        // 6.2) Return buffer to camera ASAP
        if (ioctl(fd, VIDIOC_QBUF, &b) < 0) die("VIDIOC_QBUF");

        // 6.3) Drain all available frames
        while (true) {
            MppFrame frame = nullptr;
            MPP_RET gr = mpi->decode_get_frame(ctx, &frame);
            if (gr != MPP_OK) {
                std::cerr << "decode_get_frame ret=" << gr << "\n";
                break;
            }
            if (!frame) break;

            if (mpp_frame_get_info_change(frame)) {
                on_info_change(ctx, mpi, frame);
                mpp_frame_deinit(&frame);
                continue;
            }

            int err = mpp_frame_get_errinfo(frame);
            int disc = mpp_frame_get_discard(frame);
            std::cerr << "frame: " << mpp_frame_get_width(frame) << "x"
                    << mpp_frame_get_height(frame)
                    << " err=" << err << " discard=" << disc << "\n";

            mpp_frame_deinit(&frame);
        }
    }

    std::cout << "Decoded frames: " << decoded << "\n";

    // cleanup
    mpp_packet_deinit(&pkt);
    mpi->reset(ctx);
    mpp_destroy(ctx);

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd, VIDIOC_STREAMOFF, &type);

    for (auto& mb : bufs) {
        if (mb.ptr && mb.len)
            munmap(mb.ptr, mb.len);
    }
    close(fd);
    return 0;
}
