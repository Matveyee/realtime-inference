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

    // int param = 1;
    // ret = mpi->control(ctx, MPP_DEC_SET_PARSER_SPLIT_MODE, &param);
    // if (ret) {
    //     std::cout << "set split mode failed: " << ret << std::endl;
    //     return -1;
    // }

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

    ret = mpp_buffer_group_get_external(&in_group, MPP_BUFFER_TYPE_ION);
    if (ret) {
        std::cout << "mpp_buffer_group_get_external failed: " << ret << std::endl;
        return -1;
    }

    mpp_set_log_level(3);

    ext_group_inited = false;
    ext_group = nullptr;

}

// MPP_RET MPPEntity::handle_info_change(
//     MppCtx ctx, MppApi *mpi,
//     MppFrame frame,
//     MppBufferGroup &ext_group,
//     bool &ext_inited
// ) {
//     int w  = mpp_frame_get_width(frame);
//     int h  = mpp_frame_get_height(frame);
//     int hs = mpp_frame_get_hor_stride(frame);
//     int vs = mpp_frame_get_ver_stride(frame);
//     size_t buf_size = mpp_frame_get_buf_size(frame);

//     std::cout << "INFO_CHANGE: " << w << "x" << h
//               << " stride " << hs << "x" << vs
//               << " buf_size " << buf_size << std::endl;

//     // НИЧЕГО не создаём, не коммитим, не трогаем групп
//     // Просто говорим "готов" и освобождаем frame

//     mpi->control(ctx, MPP_DEC_SET_INFO_CHANGE_READY, nullptr);
//     mpp_frame_deinit(&frame);

//     return MPP_OK;
// }

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

// int MPPEntity::put_packet_dma(int fd, int used, int total)
// {
//     MppBuffer buf      = nullptr;
//     MppBufferInfo info = {};

//     info.type = MPP_BUFFER_TYPE_EXT_DMA;
//     info.fd   = fd;      // dma-buf fd из V4L2
//     info.size = total;   // ПОЛНЫЙ размер буфера (length из QUERYBUF), а не bytesused

//     // in_group — создан ранее через mpp_buffer_group_get_external(MPP_BUFFER_TYPE_EXT_DMA)
//     MPP_RET ret = mpp_buffer_import_with_tag(
//         in_group,
//         &info,
//         &buf,
//         "v4l2_ext",
//         __FUNCTION__
//     );
//     if (ret) {
//         std::cout << "mpp_buffer_import_with_tag failed: " << ret << std::endl;
//         return -1;
//     }

//     MppPacket packet = nullptr;
//     ret = mpp_packet_init_with_buffer(&packet, buf);
//     if (ret) {
//         std::cout << "mpp_packet_init_with_buffer failed: " << ret << std::endl;
//         mpp_buffer_put(buf);
//         return -1;
//     }

//     // pos НЕ трогаем, по умолчанию = начало буфера
//     mpp_packet_set_length(packet, used);   // used = bytesused из V4L2

//     ret = mpi->decode_put_packet(ctx, packet);

//     // ВАЖНО: после этого только деинициализируем packet,
//     // buffer ref-count почистится автоматически.
//     mpp_packet_deinit(&packet);

//     if (ret && ret != MPP_ERR_BUFFER_FULL) {
//         std::cout << "decode_put_packet error: " << ret << std::endl;
//         return -1;
//     }

//     return (ret == MPP_OK) ? 0 : 1; // 1 = BUFFER_FULL, можно отреагировать снаружи
// }

int MPPEntity::put_packet_dma(int fd, int used, int total)
{

    // 3. Копируем данные
    void *buf = mmap(nullptr, total, PROT_READ, MAP_SHARED, fd, 0);

    // 4. Пакет и декод
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

        munmap(buf, total);
    return 0;
}

// int MPPEntity::put_packet_dma(int fd, int used, int total) {
//     MppBuffer buf      = nullptr;
//     MppBufferInfo info = {};

//     info.type = MPP_BUFFER_TYPE_EXT_DMA;
//     info.fd   = fd;
//     info.size = total;   // полный размер dma-buf из V4L2 (length), НЕ bytesused!


//     MPP_RET ret = mpp_buffer_import(&buf, &info);
  
//     if (ret) {
//         std::cout << "mpp_buffer_import_with_tag failed: " << ret << std::endl;
//         return -1;
//     }

//     MppPacket packet = nullptr;
    
//     ret = mpp_packet_init_with_buffer(&packet, buf);
//     if (ret) {
//         std::cout << "mpp_packet_init_with_buffer failed: " << ret << std::endl;
//         mpp_buffer_put(buf);  // packet не создался, освобождаем буфер
//         return -1;
//     }

//     put_again:
//         ret = mpi->decode_put_packet(ctx, packet);

//         if (ret == MPP_OK) {

//         } else if (ret == MPP_ERR_BUFFER_FULL) {
//             // Очистка переполненной очереди
//             while (1) {
//                 MppFrame frm = nullptr;
//                 MPP_RET r2 = mpi->decode_get_frame(ctx, &frm);
//                 if (r2 != MPP_OK || !frm)
//                     break;

//                 if (mpp_frame_get_info_change(frm)) {
//                     // Получаем первый кадр, и создаем группу буфером с соотвутствующими параметрами
//                     handle_info_change(ctx, mpi, frm, ext_group, ext_group_inited);
//                     mpp_frame_deinit(&frm);
//                     continue;
//                 }

                
//                 mpp_frame_deinit(&frm);
//             }

//             usleep(1000);
//             // Пытаемся отправить тот же самый пакет еще раз
//             goto put_again;
//         } else {
//             std::cout << "decode_put_packet error: " << ret << std::endl;
//             mpp_packet_deinit(&packet);
//             return -1;
//         }

//         mpp_packet_deinit(&packet);
//         return 0;
// }

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
            handle_info_change(ctx, mpi, frame, ext_group, ext_group_inited);
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



MppFrameQueueObject::MppFrameQueueObject(MppFrame f) {
    frame = f;
}

void MppFrameQueueObject::close() {
    mpp_frame_deinit(&frame);
}
