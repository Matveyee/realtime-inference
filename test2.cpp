#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "rk_mpi.h"
#include "mpp_packet.h"
#include "mpp_frame.h"
#include "mpp_buffer.h"
#include <string.h>
#include <errno.h>
#include <iostream>
#define MPP_ALIGN(x, a)         (((x)+(a)-1)&~((a)-1))

static unsigned char* read_file(const char* path, size_t* out_size) {
    FILE* f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "fopen('%s') failed: %s\n", path, strerror(errno));
        return NULL;
    }

    if (fseek(f, 0, SEEK_END) != 0) {
        fprintf(stderr, "fseek end failed: %s\n", strerror(errno));
        fclose(f);
        return NULL;
    }

    long sz = ftell(f);
    if (sz < 0) {
        fprintf(stderr, "ftell failed: %s\n", strerror(errno));
        fclose(f);
        return NULL;
    }

    if (sz == 0) {
        fprintf(stderr, "file '%s' is empty\n", path);
        fclose(f);
        return NULL;
    }

    rewind(f);

    unsigned char* buf = (unsigned char*)malloc((size_t)sz);
    if (!buf) {
        fprintf(stderr, "malloc(%ld) failed\n", sz);
        fclose(f);
        return NULL;
    }

    size_t rd = fread(buf, 1, (size_t)sz, f);
    if (rd != (size_t)sz) {
        fprintf(stderr, "fread got %zu/%ld bytes: %s\n",
                rd, sz, ferror(f) ? strerror(errno) : "unexpected EOF");
        free(buf);
        fclose(f);
        return NULL;
    }

    fclose(f);
    *out_size = (size_t)sz;
    return buf;
}

int main() {
    FILE* f = fopen("original.jpg", "rb");

    if (!f) {
        fprintf(stderr, "fopen('%s') failed: %s\n", "out.jpg", strerror(errno));
        return -1;
    }

    if (fseek(f, 0, SEEK_END) != 0) {
        fprintf(stderr, "fseek end failed: %s\n", strerror(errno));
        fclose(f);
        return -1;
    }

    long file_size = ftell(f);
    if (file_size < 0) {
        fprintf(stderr, "ftell failed: %s\n", strerror(errno));
        fclose(f);
        return -1;
    }

    rewind(f);

    MppCtx ctx = NULL;
    MppApi* mpi = NULL;

    int width = 1280;
    int height = 720;
    RK_U32 hor_stride = MPP_ALIGN(width, 16);
    RK_U32 ver_stride = MPP_ALIGN(height, 16);

    MppFrame  frame     = NULL;
    MppBuffer frm_buf   = NULL;
    MPP_RET ret = mpp_frame_init(&frame); /* output frame */
    if (ret) {
        printf("mpp_frame_init failed\n");
        return -1;
    }
    MppBufferGroup output_group = NULL;
    mpp_buffer_group_get_internal(&output_group, MPP_BUFFER_TYPE_ION);
    ret = mpp_buffer_group_limit_config(output_group, hor_stride * ver_stride * 4, 4);
    if (!output_group) {
        printf("failed to get buffer output_group for input frame ret %d\n", ret);
        return -1;
    }

    ret = mpp_buffer_get(output_group, &frm_buf, hor_stride * ver_stride * 4);
    if (ret) {
        printf("failed to get buffer for input frame ret %d\n", ret);
        return -1;
    }

    mpp_frame_set_buffer(frame, frm_buf);

    MppBufferGroup input_group = NULL;
    mpp_buffer_group_get_internal(&input_group, MPP_BUFFER_TYPE_ION);
    if (!input_group) {
        printf("failed to get buffer input_group for input frame ret %d\n", ret);
        return -1;
    }
    MppBuffer pkt_buf;
    mpp_buffer_get(input_group, &pkt_buf, file_size);

    void* buffer = mpp_buffer_get_ptr(pkt_buf);

    size_t read_size = fread(buffer, 1, file_size, f);

    ret = mpp_create(&ctx, &mpi);
    if (ret) { fprintf(stderr, "mpp_create failed %d\n", ret); return 1; }

    ret = mpp_init(ctx, MPP_CTX_DEC, MPP_VIDEO_CodingMJPEG);
    if (ret) { fprintf(stderr, "mpp_init failed %d\n", ret); return 1; }

    MppPacket packet;
    // mpp_packet_init(&packet, buffer, size);
    mpp_packet_init_with_buffer(&packet, pkt_buf);

    MppMeta meta = mpp_packet_get_meta(packet);
    if (meta)
        mpp_meta_set_frame(meta, KEY_OUTPUT_FRAME, frame);

    ret = mpi->decode_put_packet(ctx, packet);
    if (ret) {
        printf("%p mpp decode put packet failed ret %d\n", ctx, ret);
    }
    MppFrame frame_ret = NULL;
    std::cout << "decoding " << std::endl;
    ret = mpi->decode_get_frame(ctx, &frame_ret);
    std::cout << "decoded " << std::endl;
    if (ret || !frame_ret) {
        printf("%p mpp decode get frame failed ret %d frame %p\n", ctx, ret, frame_ret);
    }
    std::cout << frame_ret << std::endl;
    return 0;
}
