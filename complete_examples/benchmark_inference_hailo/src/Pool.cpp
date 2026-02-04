#include "Pool.hpp"

DMAPoolBufferObject::DMAPoolBufferObject(int size, int i) {

    int heap_fd = open("/dev/dma_heap/system", O_RDWR);


    // creating dma-buffer

    struct dma_heap_allocation_data req;
    memset(&req, 0, sizeof(req));
    req.len       = size;
    req.fd_flags  = O_RDWR | O_CLOEXEC;
    req.heap_flags = 0;

    if (ioctl(heap_fd, DMA_HEAP_IOCTL_ALLOC, &req) < 0) {
        perror("DMA_HEAP_IOCTL_ALLOC");
        ::close(heap_fd);
    }

    fd = req.fd;
    index = i;
    // true means available for capturing
    status = true;

}

DMAPoolBufferObject::DMAPoolBufferObject() {}

void DMAPoolBufferObject::release() {
    status = true;
}

void DMAPoolBufferObject::capture() {
    status = false;
}

void DMAPoolBufferObject::close() {
    release();
}

void DMAPoolBufferObject::destroy() {
    ::close(fd);
}