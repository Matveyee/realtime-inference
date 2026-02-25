#include "inc/rk_mpi.h"
#include "inc/mpp_log.h"
#include <iostream>
#include <fcntl.h>
#include <cstring>
#include <thread>
#include <sys/ioctl.h>
#include <linux/dma-heap.h>
#include <unistd.h>
#include <string.h>
#include <sys/mman.h>

#define PACKET_SIZE 8192
#define MPP_ALIGN(x, a)         (((x)+(a)-1)&~((a)-1))
class MPPEntity {

    public:

        MppCtx ctx;
        MppApi* mpi;
        unsigned char* buf;
        MppBufferGroup output_group;
        bool ext_group_inited;
        MppBufferGroup input_group;
        int decoded_count;

        MPPEntity();

        int init();

        MPP_RET handle_info_change(
            MppCtx ctx, MppApi *mpi,
            MppFrame frame,
            MppBufferGroup &ext_group,
            bool &ext_inited);

        int put_packet(void* buf, int size);

        int try_get_frame(MppFrame* frame);

        int put_packet_mjpg(int fd, size_t size);

        int mjpeg_mode(int w, int h);

        int mjpeg_decode(int fd, int size, MppFrame* out_frame, int w, int h);

        void notify_decoded();

        void destroy();


        
};

class MppFrameQueueObject {

    public:

        MppFrame frame;

        MppFrameQueueObject(MppFrame f);

        void close();


};