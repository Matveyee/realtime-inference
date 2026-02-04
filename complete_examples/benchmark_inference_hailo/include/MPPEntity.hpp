#include "inc/rk_mpi.h"
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

class MPPEntity {

    public:

        MppCtx ctx;
        MppApi* mpi;
        unsigned char* buf;
        MppBufferGroup ext_group;
        bool ext_group_inited;

        MPPEntity();

        int init();

        MPP_RET handle_info_change(
            MppCtx ctx, MppApi *mpi,
            MppFrame frame,
            MppBufferGroup &ext_group,
            bool &ext_inited);

        MppFrame decode_packet_dma(int fd, int size);

        int put_packet(uint8_t* buf, int size);

        int try_get_frame(MppFrame* frame);

        void destroy();

        
};

class MppFrameQueueObject {

    public:

        MppFrame frame;

        MppFrameQueueObject(MppFrame f);

        void close();


};