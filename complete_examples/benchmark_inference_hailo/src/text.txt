#include "cxxopts.hpp"
#include "utils.hpp"
#include "rockChipNN.hpp"
#include "rga/RgaUtils.h"
#include "rga/im2d.h"
#include "rga/rga.h"
#include <thread>
#include "queue.hpp"
#include <csignal>
#include <linux/dma-heap.h>
#include "MPPEntity.hpp"
#include "Pool.hpp"

#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define RESET   "\033[0m"
#define SIZE 640
#define STOP if (stop == 1) {break;}

int WIDTH;
int HEIGHT;
double FPS;
int stop = 0;
int cap_index = 0;
int conv_index = 0;
int draw_index = 0;
int End = 0;
cxxopts::ParseResult result;
MPPEntity mpp;
RockChipNN rknn;

auto simple_now() {
    return std::chrono::high_resolution_clock::now();
}

typedef std::chrono::duration<double, std::milli> Duration;

void sigint_handler(int) { 

    stop = 1;
}

class BufferQueueObject {
    
    public:

        uint8_t* data;

        BufferQueueObject(uint8_t* ptr) : data(ptr) {}

        void close() {
            free(data);
        }
};

class MPPFrameQueueObject {

    public:

        MppFrame* frame;

        MPPFrameQueueObject(MppFrame* frm) : frame(frm) {}

        void close() {
            mpp_frame_deinit(frame);
        }
};




Queue<DMAPoolBufferObject> converted_queue;
Queue<MPPFrameQueueObject> decoded_queue;
Pool<DMAPoolBufferObject, 15> nv12_pool;
Pool<DMAPoolBufferObject, 15> rgb_pool;

void start_inference() {

    while (true) {
        STOP
        if (End == 1 && converted_queue.size == 0) {
            stop = 1;
            continue;
        }
        if (converted_queue.size == 0) {
            continue;
        }

        standart_inference_ctx* ctx = new standart_inference_ctx();
        ctx->fd = converted_queue.read().fd;

        rknn.inference_dmabuf(ctx);

        converted_queue.pop();


    }

}

void start_convert() {
    
    nv12_pool.init(SIZE*SIZE*3/2);
    rgb_pool.init(SIZE*SIZE*3);

    while (true) {
        
        STOP

        if (decoded_queue.size == 0) {
            continue;
        }

        auto mfqo = decoded_queue.read();
        MppFrame* frame_ptr = mfqo.frame;
        MppFrame frame = *frame_ptr;
        if (!frame) {
            decoded_queue.pop();
            continue;
        }
       // std::cout << "In convert " <<  frame << std::endl;
        rga_buffer_t src;
        rga_buffer_t dst;
        rga_buffer_t final_dst;

        MppBuffer mpp_buffer = mpp_frame_get_buffer(frame);
        int src_fd = mpp_buffer_get_fd(mpp_buffer);

        // NV12 WIDTHxHEIGHT dma-buffer Image

        src = wrapbuffer_fd_t(
            src_fd,
            mpp_frame_get_width(frame), 
            mpp_frame_get_height(frame), 
            mpp_frame_get_hor_stride(frame), 
            mpp_frame_get_ver_stride(frame), 
            RK_FORMAT_YCbCr_420_SP);

        DMAPoolBufferObject dst_buffer = nv12_pool.capture();

        // NV12 640x640 dma-buffer Image
        dst = wrapbuffer_fd(dst_buffer.fd, SIZE, SIZE, RK_FORMAT_YCbCr_420_SP);

        IM_STATUS status = imresize(src, dst);
        
        
        decoded_queue.pop();

        DMAPoolBufferObject rgb_buffer = rgb_pool.capture();

        final_dst = wrapbuffer_fd(rgb_buffer.fd, SIZE, SIZE, RK_FORMAT_RGB_888);

        status = imcvtcolor(dst, final_dst, RK_FORMAT_YCbCr_420_SP, RK_FORMAT_RGB_888);

        dst_buffer.release();

        converted_queue.push(rgb_buffer);

    }
}



int count = 0;
void start_decode() {

    std::string filename = result["video-path"].as<std::string>();
    FILE* fp = fopen(filename.c_str(), "rb");
    if (!fp) {
        perror("fopen");
        return;
    }
    
    unsigned char *buf = (unsigned char *)malloc(PACKET_SIZE);

    bool eos = false;

    auto start = simple_now();
    auto end = simple_now();
    while (!eos) {
        STOP
        if (decoded_queue.size > 3) {
            continue;
        }
        int read_size = fread(buf, 1, PACKET_SIZE, fp);
        if (read_size <= 0) {
            // Конец файла
            std::cout << "Конец файла" << std::endl;
            eos = true;
        }
       // log("putting packet");
        int ret = mpp.put_packet(buf, read_size);
       // log("put packet");
        if (ret) {
            log("mpp.put_packet error");
            break;
        }
        MppFrame frame;
        ret = mpp.try_get_frame(&frame);
        if (ret) {
         //   log("no frame");
            continue;
        } else {
        //    std::cout << "In decode " << frame << std::endl;
            decoded_queue.push(&frame);
            cap_index++;
            end = simple_now();
            Duration dur = end - start;
            start = simple_now();
            std::cout <<"\r delay:" <<  cap_index - rknn.frames_processed << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds( (int)( (1.0 / FPS) * 1000 - dur.count() ) ) );
        }
        
    }

    End = 1;
}


int main(int argc, char* argv[]) {

    cxxopts::Options options("app", "options for app");

    options.add_options()
    ("d,debug", "Whether show debug messages or not")
    ("iw,input-width", "Input width", cxxopts::value<int>()->default_value("640"))
    ("ih,input-height", "Input height", cxxopts::value<int>()->default_value("640"))
    ("p,model-path", "Path to .hef model", cxxopts::value<std::string>())
    ("vp,video-path", "Path to video file", cxxopts::value<std::string>())
    ("f,frame-rate", "Input frame-rate", cxxopts::value<int>()->default_value("30"));

    result = options.parse(argc, argv);

    mpp.init();
    rknn.init(result["model-path"].as<std::string>());
    FPS = (double)(result["frame-rate"].as<int>());
    if (result.count("debug") != 0) {
        change_log_level(true);
    } else {
        change_log_level(false);
    }

    std::thread decode_thread(start_decode);
    std::thread convert_thread(start_convert);
    std::thread inference_thread(start_inference);

    decode_thread.join();
    convert_thread.join();
    inference_thread.join();

    double sum = 0;
    for (int i = 0; i < rknn.times.size(); i++) {

        sum += rknn.times[i];

    }
    
    std::cout << "Average Inference FPS = " << rknn.times.size() / (sum / 1000) << std::endl;

    nv12_pool.destroy();
    rgb_pool.destroy();
    mpp.destroy();

    return 0;
}

