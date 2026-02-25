#include <array>
#include <linux/dma-heap.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <cstring>
#include <iostream>
#include <thread>

class DMAPoolBufferObject {

    public:

        int fd;
        int index;
        int status;

        DMAPoolBufferObject(int size, int i);

        DMAPoolBufferObject();

        void release();

        void capture();

        void close();

};

template<typename PoolBufferObject, int size>
class Pool {

    public:

        std::array<PoolBufferObject, size> array;

        Pool(int buffer_size);

        Pool();

        void init(int buffer_size);

        void destroy();

        PoolBufferObject capture();

};

template<typename PoolBufferObject, int size>
Pool<PoolBufferObject, size>::Pool(int buffer_size) {

    for (int i = 0; i < size; i++) {
        PoolBufferObject pbo(buffer_size, i);
        array[i] = pbo;
    }

}

template<typename PoolBufferObject, int size>
Pool<PoolBufferObject, size>::Pool() {}

template<typename PoolBufferObject, int size>
void Pool<PoolBufferObject, size>::init(int buffer_size) {

    for (int i = 0; i < size; i++) {
        PoolBufferObject pbo(buffer_size, i);
        array[i] = pbo;
    }

}


template<typename PoolBufferObject, int size>
PoolBufferObject Pool<PoolBufferObject, size>::capture() {

    int index = 0;
    auto start = std::chrono::high_resolution_clock::now();
    while (true) {
        
        PoolBufferObject pbo = array[index];

        if (pbo.status) {
            pbo.capture();
            return pbo;
        } else {
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration = end - start;
            if (duration.count() > 100) {
                std::cout << "too long capturing buffer" << std::endl;
            }
            if (index == size - 1) {
                index = -1;
            }
            index++;
        }

    }
    
}

template<typename PoolBufferObject, int size>
void Pool<PoolBufferObject, size>::destroy() {
    
    for (int i = 0; i < size; i++) {
        array[i].destroy();
    }

}


