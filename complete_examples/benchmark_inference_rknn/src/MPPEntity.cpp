#include "MPPEntity.hpp"

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

    int param = 1;
    ret = mpi->control(ctx, MPP_DEC_SET_PARSER_SPLIT_MODE, &param);
    if (ret) {
        std::cout << "set split mode failed: " << ret << std::endl;
        return -1;
    }

    ret = mpp_init(ctx, MPP_CTX_DEC, MPP_VIDEO_CodingAVC);

    if (ret) {
        std::cout << "mpp_init failed: " << ret << std::endl;
        return -1;
    }

    buf = (unsigned char *)malloc(PACKET_SIZE);

    if (!buf) {
        std::cout << "malloc failed" << std::endl;
        return -1;
    }

    ext_group_inited = false;
    ext_group = nullptr;

}

void MPPEntity::destroy() {
    mpp_destroy(ctx);
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

MppFrame MPPEntity::decode_packet_dma(int fd, int size) {

    
    MppPacket packet = nullptr;
    MppBuffer buf = nullptr;
    MppBufferInfo info;

    info.type = MPP_BUFFER_TYPE_EXT_DMA;
    info.fd = fd;
    info.size = size;

    MPP_RET ret = mpp_buffer_import(&buf, &info);

    if (ret) {
        std::cout << "mpp_buffer_import failed: " << ret << std::endl;
    }





    ret = mpp_packet_init_with_buffer(&packet, buf);
    if (ret) {
        std::cout << "mpp_packet_init failed: " << ret << std::endl;
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
                handle_info_change(ctx, mpi, frm, ext_group, ext_group_inited);
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
    }

        mpp_packet_deinit(&packet);


    // После КАЖДОГО успешного put – читаем все доступные кадры
    int br = 0;
    while (1) {
        MppFrame frame = nullptr;
        ret = mpi->decode_get_frame(ctx, &frame);
        if (ret != MPP_OK)
            break;
        if (!frame)
            break;

        if (mpp_frame_get_info_change(frame)) {
            handle_info_change(ctx, mpi, frame, ext_group, ext_group_inited);
            mpp_frame_deinit(&frame);
            continue;
        }

        
        if (!mpp_frame_get_errinfo(frame)) {

            return frame;

        }

        // mpp_frame_deinit(&frame);
    }
    if (br == 1) {
    }
}

int MPPEntity::put_packet(uint8_t* buf, int size) {

    MppPacket packet = nullptr;


    MPP_RET ret = mpp_packet_init(&packet, buf, PACKET_SIZE);
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
                handle_info_change(ctx, mpi, frm, ext_group, ext_group_inited);
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

int MPPEntity::try_get_frame(MppFrame* source_frame) {

    while (1) {
        MppFrame frame = nullptr;
        MPP_RET ret = mpi->decode_get_frame(ctx, &frame);
        if (ret != MPP_OK) {
            return -1;
            break;
        }
        if (!frame) { 
            return -1;
            break;
        }
        if (mpp_frame_get_info_change(frame)) {
            handle_info_change(ctx, mpi, frame, ext_group, ext_group_inited);
            mpp_frame_deinit(&frame);
            continue;
        }

        
        if (!mpp_frame_get_errinfo(frame)) {

            *source_frame = frame;
            return 0;

        }
    }

}


MppFrameQueueObject::MppFrameQueueObject(MppFrame f) {
    frame = f;
}

void MppFrameQueueObject::close() {
    mpp_frame_deinit(&frame);
}
