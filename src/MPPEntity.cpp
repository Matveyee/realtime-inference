#include "MPPEntity.hpp"
#include "utils.hpp"



MPPEntity::MPPEntity() {

    ctx = nullptr;
    mpi = nullptr;

}
int MPPEntity::init() { 

    int ret = mpp_create(&ctx, &mpi);

    if (ret) {
        std::cout << "mpp_create failed: " << ret << std::endl;
        return -1;
    }

    ret = mpp_init(ctx, MPP_CTX_DEC, MPP_VIDEO_CodingMJPEG);

    if (ret) {
        std::cout << "mpp_init failed: " << ret << std::endl;
        return -1;
    }

    buf = (unsigned char *)malloc(PACKET_SIZE);

    if (!buf) {
        std::cout << "malloc failed" << std::endl;
        return -1;
    }

    ret = mpp_buffer_group_get_external(&input_group, MPP_BUFFER_TYPE_ION);
    if (ret) {
        std::cout << "mpp_buffer_group_get_external failed: " << ret << std::endl;
        return -1;
    }

    MppFrameFormat out_fmt = MPP_FMT_RGB888; // NV12
    mpi->control(ctx, MPP_DEC_SET_OUTPUT_FORMAT, &out_fmt);

    mpp_set_log_level(3);

    ext_group_inited = false;
    output_group = nullptr;

}


MPP_RET MPPEntity::handle_info_change(
    
    MppCtx ctx, MppApi *mpi,
    MppFrame frame,
    MppBufferGroup &ext_group,
    bool &ext_inited

    ) {

    int w  = mpp_frame_get_width(frame);
    int h  = mpp_frame_get_height(frame);
    int hs = mpp_frame_get_hor_stride(frame);
    int vs = mpp_frame_get_ver_stride(frame);
    size_t buf_size = mpp_frame_get_buf_size(frame);

    std::cout << "INFO_CHANGE: " << w << "x" << h
              << " stride " << hs << "x" << vs
              << " buf_size " << buf_size << std::endl;

    if (!ext_inited) {
        // 1) создаём внешнюю группу
        MPP_RET ret = mpp_buffer_group_get_external(&ext_group, MPP_BUFFER_TYPE_DMA_HEAP);
        if (ret) {
            std::cout << "mpp_buffer_group_get_external failed: " << ret << std::endl;
            return ret;
        }

        int buffer_count = 50;
        int heap_fd = open("/dev/dma_heap/system", O_RDWR);
        if (heap_fd < 0) {
            perror("open /dev/dma_heap/system");
            return MPP_NOK;
        }

        for (int i = 0; i < buffer_count; i++) {
            struct dma_heap_allocation_data req;
            memset(&req, 0, sizeof(req));
            req.len       = buf_size;
            req.fd_flags  = O_RDWR | O_CLOEXEC;

            if (ioctl(heap_fd, DMA_HEAP_IOCTL_ALLOC, &req) < 0) {
                perror("DMA_HEAP_IOCTL_ALLOC");
                close(heap_fd);
                return MPP_NOK;
            }

            int fd = req.fd;

            MppBufferInfo info;
            memset(&info, 0, sizeof(info));
            info.type  = MPP_BUFFER_TYPE_DMA_HEAP;
            info.size  = buf_size;
            info.fd    = fd;
            info.index = i;

            MppBuffer mbuf;
            ret = mpp_buffer_import(&mbuf, &info);
            if (ret) {
                std::cout << "mpp_buffer_import failed: " << ret << std::endl;
                close(heap_fd);
                return ret;
            }

            ret = mpp_buffer_commit(ext_group, &info);
            if (ret) {
                std::cout << "mpp_buffer_commit failed: " << ret << std::endl;
                close(heap_fd);
                return ret;
            }
        }

        close(heap_fd);

        // 3) Сообщаем декодеру
        MPP_RET ret2 = mpi->control(ctx, MPP_DEC_SET_EXT_BUF_GROUP, ext_group);
        if (ret2) {
            std::cout << "MPP_DEC_SET_EXT_BUF_GROUP failed: " << ret2 << std::endl;
            return ret2;
        }

        ext_inited = true;
    }

    // 4) Подтверждаем info_change
    mpi->control(ctx, MPP_DEC_SET_INFO_CHANGE_READY, NULL);
    return MPP_OK;

}


int MPPEntity::put_packet(void* buf, int size) {

    MppPacket packet = nullptr;


    MPP_RET ret = mpp_packet_init(&packet, buf, size);
    if (ret) {
        std::cout << "mpp_packet_init failed: " << ret << std::endl;
        return -1;
    }

    put_again:

    ret = mpi->decode_put_packet(ctx, packet);

    if (ret == MPP_OK) {
        // пакет успешно принят
    } else if (ret == MPP_ERR_BUFFER_FULL) {
        // Очередь входных пакетов переполнена – нужно вытащить кадры
        while (1) {
            MppFrame frm = nullptr;
            MPP_RET r2 = mpi->decode_get_frame(ctx, &frm);
            if (r2 != MPP_OK || !frm)
                break;

            if (mpp_frame_get_info_change(frm)) {
                handle_info_change(ctx, mpi, frm, output_group, ext_group_inited);
                mpp_frame_deinit(&frm);
                continue;
            }

            // Здесь можно обработать кадр (но сейчас просто освобождаем)
            mpp_frame_deinit(&frm);
        }

        usleep(1000);
        goto put_again;
    } else {
        std::cout << "decode_put_packet error: " << ret << std::endl;
        mpp_packet_deinit(&packet);
        return -1;
    }

        mpp_packet_deinit(&packet);
    return 0;

}

int MPPEntity::try_get_frame(MppFrame* out_frame) {
    int i = 0;
    while (true) {
    
    
        MppFrame frame = nullptr;
        MPP_RET ret = mpi->decode_get_frame(ctx, &frame);

        if (ret != MPP_OK) {
            // настоящая ошибка
            error("decode_get_frame: ", ret);
            return -2;
        }

        if (!frame) {
            // просто нет готового кадра сейчас
            i++;
            if (i > 200) {
                return -1;
            }
            continue;
        }

        if (mpp_frame_get_info_change(frame)) {
            handle_info_change(ctx, mpi, frame, output_group, ext_group_inited);
            mpp_frame_deinit(&frame);
            return 1; // это не ошибка – просто сервисное событие
        }

        if (mpp_frame_get_errinfo(frame)) {
            // битый кадр – выкидываем
            mpp_frame_deinit(&frame);
            return 1;
        }
        *out_frame = frame;
    }
     // ответственность за mpp_frame_deinit на вызывающем коде
    return 0;           // 0 – есть валидный кадр
}

int MPPEntity::mjpeg_mode(int w, int h) {

    RK_U32 hor_stride = MPP_ALIGN(w, 16);
    RK_U32 ver_stride = MPP_ALIGN(h, 16);

    // mpp_buffer_group_get_external(&output_group, MPP_BUFFER_TYPE_DMA_HEAP);
    mpp_buffer_group_get_internal(&output_group, MPP_BUFFER_TYPE_ION);
    MPP_RET ret = mpp_buffer_group_limit_config(output_group, hor_stride * ver_stride * 4, 30);
    if (!output_group) {
        printf("failed to get buffer output_group for input frame ret %d\n", ret);
        return -1;
    }

    // int buffer_count = 10;
    // int heap_fd = open("/dev/dma_heap/system", O_RDWR);
    // if (heap_fd < 0) {
    //     perror("open /dev/dma_heap/system");
    //     return MPP_NOK;
    // }

    // for (int i = 0; i < buffer_count; i++) {
    //     struct dma_heap_allocation_data req;
    //     memset(&req, 0, sizeof(req));
    //     req.len       = hor_stride * ver_stride * 4;
    //     req.fd_flags  = O_RDWR | O_CLOEXEC;

    //     if (ioctl(heap_fd, DMA_HEAP_IOCTL_ALLOC, &req) < 0) {
    //         perror("DMA_HEAP_IOCTL_ALLOC");
    //         close(heap_fd);
    //         return MPP_NOK;
    //     }

    //     int fd = req.fd;

    //     MppBufferInfo info;
    //     memset(&info, 0, sizeof(info));
    //     info.type  = MPP_BUFFER_TYPE_DMA_HEAP;
    //     info.size  = hor_stride * ver_stride * 4;
    //     info.fd    = fd;
    //     info.index = i;

    //     MppBuffer mbuf;
    //     ret = mpp_buffer_import(&mbuf, &info);
    //     if (ret) {
    //         std::cout << "mpp_buffer_import failed: " << ret << std::endl;
    //         close(heap_fd);
    //         return ret;
    //     }

    //     ret = mpp_buffer_commit(output_group, &info);
    //     if (ret) {
    //         std::cout << "mpp_buffer_commit failed: " << ret << std::endl;
    //         close(heap_fd);
    //         return ret;
    //     }
    // }

    // close(heap_fd);

    // MPP_RET ret2 = mpi->control(ctx, MPP_DEC_SET_EXT_BUF_GROUP, output_group);
    // if (ret2) {
    //     std::cout << "MPP_DEC_SET_EXT_BUF_GROUP failed: " << ret2 << std::endl;
    //     return ret2;
    // }

    
    mpp_buffer_group_get_internal(&input_group, MPP_BUFFER_TYPE_ION);

}

int MPPEntity::mjpeg_decode(int fd, int size, MppFrame* out_frame, int w, int h) {
        if (decoded_count == 30) {
            return 1;
        }
        MppBuffer pkt_buf;
        MppBufferInfo info;
        memset(&info, 0, sizeof(info));
        info.type  = MPP_BUFFER_TYPE_DMA_HEAP;
        info.size  = size;
        info.fd    = fd;
        info.index = 1;
        // std::cout << "importing buffer" << std::endl;
        int ret = mpp_buffer_import(&pkt_buf, &info);
        if (ret) {
            std::cout << "mpp_buffer_import failed: " << ret << std::endl;
            return -1;
        }
        // std::cout << "importi succeed" << std::endl;
        if (ret) {
            std::cout << "mpp_buffer_import failed: " << ret << std::endl;
            return -1;
        }

        
        MppFrame  frame     = NULL;
        MppBuffer frm_buf   = NULL;
        ret = mpp_frame_init(&frame); /* output frame */
        if (ret) {
            printf("mpp_frame_init failed\n");
            return -1;
        }

        RK_U32 hor_stride = MPP_ALIGN(w, 16);
        RK_U32 ver_stride = MPP_ALIGN(h, 16);

        
        ret = mpp_buffer_get(output_group, &frm_buf, hor_stride * ver_stride * 4);
        if (ret) {
            printf("failed to get buffer for input frame ret %d\n", ret);
            return -1;
        }

        mpp_frame_set_buffer(frame, frm_buf);

        MppPacket packet;
        mpp_packet_init_with_buffer(&packet, pkt_buf);

        //// Important! ////    ////
        MppMeta meta = mpp_packet_get_meta(packet);
        if (meta)
            mpp_meta_set_frame(meta, KEY_OUTPUT_FRAME, frame);
        ////    ////    ////    ////    

        ret = mpi->decode_put_packet(ctx, packet);
        if (ret) {
            printf("%p mpp decode put packet failed ret %d\n", ctx, ret);
            return -1;
        }
        MppFrame frame_ret = NULL;
        std::cout << "decoding " << std::endl;

        ret = mpi->decode_get_frame(ctx, &frame_ret);
        int fmt_raw = mpp_frame_get_fmt(frame);
        int fmt_id  = fmt_raw & MPP_FRAME_FMT_COLOR_MASK;
        fprintf(stderr, "got frame fmt_id=%d\n", (fmt_raw));
        if (ret || !frame_ret) {
            printf("%p mpp decode get frame failed ret %d frame %p\n", ctx, ret, frame_ret);
            return -1;
        }
        decoded_count++;
        *out_frame = frame_ret;
        frame_ret = NULL;
        std::cout << "DECODED " << std::endl;
        std::cout << out_frame << std::endl;
        return 0;

}

void MPPEntity::notify_decoded() {
    decoded_count--;
}

MppFrameQueueObject::MppFrameQueueObject(MppFrame f) {
    frame = f;
}

void MppFrameQueueObject::close() {
    mpp_buffer_put(mpp_frame_get_buffer(frame));
    mpp_frame_deinit(&frame);
}

