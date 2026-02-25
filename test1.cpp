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
#include <linux/dma-heap.h>



extern "C" {
#include "include/inc/rk_mpi.h"
#include "include/inc/mpp_packet.h"
#include "include/inc/mpp_frame.h"
#include "include/inc/mpp_log.h"
// #include "mpp_dec_cfg.h"
}

#define MPP_ALIGN(x, a)         (((x)+(a)-1)&~((a)-1))

int width;
int height;

MPP_RET create_buffer_group(
    
    MppCtx ctx, MppApi *mpi,
    MppBufferGroup &ext_group

    ) {

    int w  = width;
    int h  = height;
    int hs = MPP_ALIGN(width, 16);
    int vs = MPP_ALIGN(height, 16);


    MPP_RET ret = mpp_buffer_group_get_external(&ext_group, MPP_BUFFER_TYPE_DMA_HEAP);
    if (ret) {
        std::cout << "mpp_buffer_group_get_external failed: " << ret << std::endl;
        return ret;
    }

    int buffer_count = 10;
    int heap_fd = open("/dev/dma_heap/system", O_RDWR);
    if (heap_fd < 0) {
        perror("open /dev/dma_heap/system");
        return MPP_NOK;
    }

    for (int i = 0; i < buffer_count; i++) {
        struct dma_heap_allocation_data req;
        memset(&req, 0, sizeof(req));
        req.len       = hs * vs * 4;
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
        info.size  = hs * vs * 4;
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


    // 4) Подтверждаем info_change
    // mpi->control(ctx, MPP_DEC_SET_INFO_CHANGE_READY, NULL);
    return MPP_OK;

}

int main() {

    FILE* file = fopen("../ffmpeg-rockchip/out.jpg", "rb");
    

    fseek(file, 0, SEEK_END);
    long size = ftell(file);
    rewind(file);

    // Выделяем память
    unsigned char *buffer = (unsigned char*)malloc(size);
    if (!buffer) {
        perror("Ошибка malloc");
        fclose(file);
        return 1;
    }

    // Читаем файл
    fread(buffer, 1, size, file);
    fclose(file);

    MppCtx ctx          = NULL;
    MppApi *mpi         = NULL;

    width = 1280;
    height = 720;

    RK_U32 hs = MPP_ALIGN(width, 16);
    RK_U32 vs = MPP_ALIGN(height, 16);

    int ret = mpp_create(&ctx, &mpi);
    mpp_err_f("errorwerwer");
    if (ret) {
        std::cout << "mpp_create failed: " << ret << std::endl;
        return -1;
    }

    ret = mpp_init(ctx, MPP_CTX_DEC, MPP_VIDEO_CodingMJPEG);

    if (ret) {
        std::cout << "mpp_init failed: " << ret << std::endl;
        // return -1;
    }

    MppBufferGroup ext_group;

    create_buffer_group(ctx, mpi, ext_group);
    

    MppBuffer frm_buf;
    MppFrame frame = NULL;
    mpp_frame_init(&frame);
    ret = mpp_buffer_get(ext_group, &frm_buf, hs * vs * 4);
    if (ret) {
        printf("failed to get buffer for input frame ret %d\n", ret);

    }

    RK_U32 block = 1;
    mpi->control(ctx, MPP_SET_INPUT_BLOCK, &block);
    mpi->control(ctx, MPP_SET_OUTPUT_BLOCK, &block);


    mpp_frame_set_buffer(frame, frm_buf);

    MppPacket packet;

    ret = mpp_packet_init(&packet, buffer, size);
    mpp_packet_set_length(packet, size);

    mpi->poll(ctx, MPP_PORT_INPUT, MPP_POLL_BLOCK);
    MppMeta meta = mpp_packet_get_meta(packet);
    if (meta)
        mpp_meta_set_frame(meta, KEY_OUTPUT_FRAME, frame);
    try_again:

    ret = mpi->decode_put_packet(ctx, packet);
    if (ret) {
        printf("%p mpp decode put packet failed ret %d\n", ctx, ret);
        usleep(1000 * 2);
        
        // return -1;
    }
    MppFrame frame_ret = NULL;
    
    ret = mpi->decode_get_frame(ctx, &frame_ret);
    if (ret) {
        printf("%p mpp decode get frame failed ret %d frame %p\n", ctx, ret, frame_ret);
        goto try_again;

    }
    if (!frame_ret) {
        std::cout << "NULL" << std::endl;
    }
    std::cout << "decoded frame" << std::endl;
}