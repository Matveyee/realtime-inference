#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include "rk_mpi.h"
#include "mpp_packet.h"
#include "mpp_frame.h"
#include "mpp_buffer.h"

static int read_whole(const char* path, unsigned char** out, size_t* out_sz) {
    FILE* f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "fopen: %s\n", strerror(errno)); return -1; }

    if (fseek(f, 0, SEEK_END) != 0) { perror("fseek"); fclose(f); return -1; }
    long sz = ftell(f);
    if (sz <= 0) { fprintf(stderr, "ftell=%ld\n", sz); fclose(f); return -1; }
    rewind(f);

    unsigned char* buf = (unsigned char*)malloc((size_t)sz);
    if (!buf) { perror("malloc"); fclose(f); return -1; }

    size_t rd = fread(buf, 1, (size_t)sz, f);
    fclose(f);
    if (rd != (size_t)sz) { fprintf(stderr, "short read %zu/%ld\n", rd, sz); free(buf); return -1; }

    // Быстрая sanity-проверка JPEG SOI
    if (!(rd >= 2 && buf[0] == 0xFF && buf[1] == 0xD8)) {
        fprintf(stderr, "not a JPEG? first bytes: %02X %02X\n", buf[0], buf[1]);
        free(buf);
        return -1;
    }

    *out = buf;
    *out_sz = rd;
    return 0;
}

int main() {
    unsigned char* data = NULL;
    size_t size = 0;
    if (read_whole("original.jpg", &data, &size) != 0) return 1;

    MppCtx ctx = NULL;
    MppApi* mpi = NULL;

    MPP_RET ret = mpp_create(&ctx, &mpi);
    if (ret) { fprintf(stderr, "mpp_create %d\n", ret); return 1; }

    ret = mpp_init(ctx, MPP_CTX_DEC, MPP_VIDEO_CodingMJPEG);
    if (ret) { fprintf(stderr, "mpp_init %d\n", ret); return 1; }

    // Таймауты вместо deprecated block (если команды доступны в твоих headers — оставь)
    RK_S32 tmo = 100;
    mpi->control(ctx, MPP_SET_INPUT_TIMEOUT,  &tmo);
    mpi->control(ctx, MPP_SET_OUTPUT_TIMEOUT, &tmo);

    MppPacket pkt = NULL;
    mpp_packet_init(&pkt, data, size);
    mpp_packet_set_length(pkt, (RK_U32)size);

    // put
    ret = mpi->decode_put_packet(ctx, pkt);
    if (ret) { fprintf(stderr, "put_packet %d\n", ret); return 1; }

    // get (крутимся, пока не придёт кадр или ошибка)
    MppFrame frame = NULL;
    for (int tries = 0; tries < 50; ++tries) {
        mpi->poll(ctx, MPP_PORT_OUTPUT, MPP_POLL_BLOCK);
        ret = mpi->decode_get_frame(ctx, &frame);
        if (ret) { fprintf(stderr, "get_frame %d\n", ret); break; }
        if (frame) break;
    }

    if (!frame) {
        fprintf(stderr, "no frame produced\n");
    } else {
        if (mpp_frame_get_errinfo(frame) || mpp_frame_get_discard(frame))
            fprintf(stderr, "frame err/discard\n");
        else
            printf("OK decoded: %dx%d fmt=%d\n",
                   mpp_frame_get_width(frame),
                   mpp_frame_get_height(frame),
                   mpp_frame_get_fmt(frame));

        mpp_frame_deinit(&frame);
    }

    mpp_packet_deinit(&pkt);
    mpp_destroy(ctx);
    free(data);
    return 0;
}