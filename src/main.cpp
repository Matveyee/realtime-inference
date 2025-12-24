#include "camera.hpp"
#include "relay.hpp"
// #include "../include/modbus.hpp"
#include "GxIAPI.h"
#include "DxImageProc.h"
#include "hailoNN.hpp"
#include "rockChipNN.hpp"
#include "utils.hpp"
#include "rga/RgaUtils.h"
#include "rga/im2d.h"
#include "rga/rga.h"
#include "gpio.hpp"
#include <thread>
#include "queue.hpp"
#include <csignal>
#include <linux/dma-heap.h>

#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define RESET   "\033[0m"

int WIDTH;
int HEIGHT;
int FPS;
int sizee;
std::mutex main_mutex;
AbstractNNBase* NN = NULL;
void* context = NULL;
int stop = 0;
#define STOP if (stop == 1) {break;}
int cap_index = 0;
int conv_index = 0;
int draw_index = 0;
GPIO gpio;
auto simple_now() {
    return std::chrono::high_resolution_clock::now();
}

typedef std::chrono::duration<double, std::milli> Duration;

void sigint_handler(int) { 
  //  main_mutex.unlock();
    stop = 1;
}

class FrameBufferQueueObject {

    public:

        PGX_FRAME_BUFFER* pFrameBuffer;

        AICamera* cam;

        int i;
        FrameBufferQueueObject(PGX_FRAME_BUFFER* pFrameBuffer_param, AICamera* cam_param, int index) : pFrameBuffer(pFrameBuffer_param), cam(cam_param), i(index) {}

        void close() {
            cam->returnBuffer(pFrameBuffer);
            log("Returned");
        }


};

class BufferQueueObject {
    
    public:

        uint8_t* data;

        BufferQueueObject(uint8_t* ptr) : data(ptr) {}

        void close() {
            free(data);
        }
};

Queue<FrameBufferQueueObject> queue;
Queue<BufferQueueObject> pic_queue;

void start_inference() {

}

void start_draw() {
    while (true) {
        STOP
        if (pic_queue.size <= 1) {
          //  std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        uint8_t* data = pic_queue.read().data;
        drawPicture(data, WIDTH, HEIGHT);
        draw_index++;
        if (draw_index % 10 == 0)
            std::cout << "DELAY: " << cap_index <<"; " << conv_index << "; " << draw_index <<"; "<< cap_index - draw_index << std::endl;
        pic_queue.pop();

        STOP
    }
}
void start_convert() {
    while (true) {
        STOP
        if (queue.size <= 2) {
         //   std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // if (pic_queue.size >= 3) {
        //     std::this_thread::sleep_for(std::chrono::milliseconds(2));
        //     continue;
        // }
        
        std::cout << querystring(RGA_MAX_INPUT) << std::endl;

        log(std::to_string(queue.size));
        FrameBufferQueueObject fbqo = queue.read();
      
        PGX_FRAME_BUFFER buf = *fbqo.pFrameBuffer;


        uint8_t* data = (uint8_t*)malloc(buf->nWidth * buf->nHeight * 3);
        if (data == NULL)
        {
        return ;
        }
        else
        {

        memset(data,0,buf->nWidth * buf->nHeight * 3);
        }

        DX_BAYER_CONVERT_TYPE cvtype
        = RAW2RGB_NEIGHBOUR;
        DX_PIXEL_COLOR_FILTER nBayerType = BAYERRG;
        bool bFlip = true;

        VxInt32 DxStatus = DxRaw8toRGB24(buf->pImgBuf,data,buf->nWidth,
        buf->nHeight,cvtype,nBayerType,bFlip);

        uint8_t* dst_buf = (uint8_t*)malloc(WIDTH * HEIGHT * 3);
        uint8_t* infer_buf = (uint8_t*)malloc(640 * 640 * 3);
        rga_buffer_t src; rga_buffer_t dst;
         rga_buffer_t infer;
        rga_buffer_handle_t src_handle; rga_buffer_handle_t dst_handle; 
         rga_buffer_handle_t infer_handle;

        src_handle = importbuffer_virtualaddr(data, buf->nWidth, buf->nHeight, RK_FORMAT_RGB_888);
        dst_handle = importbuffer_virtualaddr(dst_buf, WIDTH, HEIGHT, RK_FORMAT_RGB_888);
        

        src = wrapbuffer_handle(
            src_handle,
            buf->nWidth,
            buf->nHeight,
            RK_FORMAT_RGB_888
        );

        dst = wrapbuffer_handle(
            dst_handle,
            WIDTH,
            HEIGHT,
            RK_FORMAT_RGB_888
        );

        

        // src = wrapbuffer_virtualaddr(data, buf->nWidth, buf->nHeight, RK_FORMAT_RGB_888);
        // dst = wrapbuffer_virtualaddr(dst_buf, WIDTH, HEIGHT, RK_FORMAT_RGB_888);
        
        IM_STATUS STATUS = imresize(src, dst);
        std::cout << "STATUS : " << STATUS << std::endl;

        infer_handle = importbuffer_virtualaddr(infer_buf, 640, 640, RK_FORMAT_RGB_888);

        infer = wrapbuffer_handle(
            infer_handle,
            640,
            640,
            RK_FORMAT_RGB_888
        );

       
     //   rga_buffer_t infer;
        //  infer = wrapbuffer_virtualaddr(infer_buf, 640, 640, RK_FORMAT_RGB_888);

        STATUS = imresize(dst, infer);
        std::cout << "STATUS : " << STATUS << std::endl;

        STATUS = releasebuffer_handle(src_handle);
        STATUS = releasebuffer_handle(infer_handle);
        STATUS = releasebuffer_handle(dst_handle);

        free(data);
        //free(infer_buf);

            

        queue.pop();
        BufferQueueObject bqo(dst_buf);
        pic_queue.push(dst_buf);       
        conv_index++; 
        standart_inference_ctx* ctx = new standart_inference_ctx();
        ctx->input_buffer = infer_buf;

        context = ctx;
        NN->inference(context);

        STOP
    }
}

int count = 0;
void start_capture(AICamera& cam) {
    
    while (true) {
        STOP
        
        if (queue.size >= 3) {
          //  std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        auto start = simple_now();
        PGX_FRAME_BUFFER pFrameBuffer;
        
        cam.startStream();

        cam.getBuffer(&pFrameBuffer);
        std::cout << YELLOW << "Captured" << RESET << std::endl;
        
        FrameBufferQueueObject fbqo(&pFrameBuffer, &cam, count);
        printf("before %p\n", pFrameBuffer);
        std::cout << BLUE << "INDEX " << count << RESET<< std::endl;
        count++;
        queue.push(fbqo);
        cap_index++;
        auto end = simple_now();
        Duration dur = end - start;

        std::this_thread::sleep_for(std::chrono::milliseconds( (int)((1.0 / FPS) * 1000) - (int)dur.count()));


        

        STOP
    }
    
}

int main(int argc, char* argv[]) {

 //   main_mutex.lock();

    
    int drm_fd = open("/dev/dri/card0", O_RDWR | O_CLOEXEC);
    drm_init(drm_fd);



    // Relay relay("/dev/ttyUSB0");
    // gpio.init(39);
    signal(SIGINT, sigint_handler);
    AICamera cam;
    sizee = std::stoi(argv[1]);
    cam.setHeight(sizee);
    cam.setWidth(sizee);
    WIDTH = HEIGHT = std::stoi(argv[2]);
    FPS = std::stoi(argv[3]);
    cam.setOffsetX( (2592 - sizee) / 2 );
    cam.setOffsetY( (1944 - sizee) / 2 );

    if ( std::string(argv[4]) == "hailo") {
        HailoNN* hailo = new HailoNN(argv[5]);
        NN = hailo;

    }
    if ( std::string(argv[4]) == "rockchip") {
        RockChipNN* rockchip = new RockChipNN(argv[5]);
        NN = rockchip;

    }
    auto start = std::chrono::high_resolution_clock::now();
    std::thread capture_thread(start_capture, std::ref(cam));
    std::thread convert_thread(start_convert);
    std::thread draw_thread(start_draw);


    capture_thread.join();
    convert_thread.join();
    draw_thread.join();
    
    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> dura = end - start;

    std::cout << "AVERAGE FPS = " << count / (dura.count() / 1000) << std::endl;
    cam.destroy();

    drm_destroy(drm_fd);

    return 0;
}

