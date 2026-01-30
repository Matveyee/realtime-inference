#include "camera.hpp"
#include "relay.hpp"
// #include "../include/modbus.hpp"
#include "cxxopts.hpp"
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
double FPS;
int sizee;
std::mutex main_mutex;
std::mutex capture_mutex;
std::mutex convert_mutex;
std::mutex draw_mutex;
AbstractNNBase* NN = NULL;
void* context = NULL;
int stop = 0;
#define STOP if (stop == 1) {break;}
int cap_index = 0;
int conv_index = 0;
int draw_index = 0;
GPIO gpio;
cxxopts::ParseResult result;
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
Queue<BufferQueueObject> infer_queue;
Queue<FrameBufferQueueObject> free_queue;

void start_inference() {
    while (true) {
        STOP

        if (infer_queue.size == 0) {
        //    std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        standart_inference_ctx* ctx = new standart_inference_ctx();
        ctx->input_buffer = infer_queue.read().data;

        context = ctx;
        NN->inference(context);
        infer_queue.pop();

    }
}

void start_draw() {
    while (true) {
        STOP
        if (pic_queue.size <= 0) {
        //    std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        uint8_t* data = pic_queue.read().data;
        drawPicture(data, WIDTH, HEIGHT);
        draw_index++;
        std::cout << "index " << cap_index << std::endl;
        if (cap_index % 10 == 0){ 
            std::cout << "DELAY: " << cap_index <<"; " << conv_index << "; " << draw_index <<"; "<< cap_index - draw_index << std::endl;
        }

        infer_queue.push(data);
        pic_queue.remove();
        std::cout << "size before" << pic_queue.size << std::endl; 
        
        std::cout << "size after" << pic_queue.size << std::endl;

        STOP
    }
}
void start_convert() {
    
    while (true) {
        STOP
        if (queue.size <= 0) {
          //  std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        
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
        bool bFlip = false;

        VxInt32 DxStatus = DxRaw8toRGB24(buf->pImgBuf,data,buf->nWidth,
        buf->nHeight,cvtype,nBayerType,bFlip);
        
        if (result.count("noresize") == 0) {

        uint8_t* dst_buf = (uint8_t*)malloc(WIDTH * HEIGHT * 3);
        rga_buffer_t src; rga_buffer_t dst;
        rga_buffer_handle_t src_handle; rga_buffer_handle_t dst_handle; 
         
        

        
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

    
        
        IM_STATUS STATUS = imresize(src, dst);
        free(data);
        STATUS = releasebuffer_handle(src_handle);
        
        pic_queue.push(dst_buf);
        STATUS = releasebuffer_handle(dst_handle);
        }
        else {
            pic_queue.push(data);
        }  
       // free(data);
        // std::cout << "STATUS : " << STATUS << std::endl;

        // uint8_t* infer_buf = (uint8_t*)malloc(640 * 640 * 3);
        // rga_buffer_handle_t infer_handle;
        // infer_handle = importbuffer_virtualaddr(infer_buf, 640, 640, RK_FORMAT_RGB_888);
        // rga_buffer_t infer;


        // infer = wrapbuffer_handle(
        //     infer_handle,
        //     640,
        //     640,
        //     RK_FORMAT_RGB_888
        // );


        // STATUS = imresize(dst, infer);
        //STATUS = releasebuffer_handle(infer_handle);
       // STATUS = releasebuffer_handle(dst_handle);
        
        // free_queue.push(fbqo);
        // queue.remove();
        queue.pop();
        
        conv_index++;
      //  infer_queue.push(infer_buf);
        

        STOP
    }
}

int count = 0;
void start_capture(AICamera& cam) {
    auto start = simple_now();
    // queue.set_limit(4);
    while (true) {
        STOP
        
        if (queue.size >= 4) {
            // std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
            // log("MUTEX: blocked in capture thread");
            // queue.size_mutex.lock();
        }
        
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
        start = simple_now();
        
        std::this_thread::sleep_for(std::chrono::milliseconds( (int)( (1.0 / FPS) * 1000 - dur.count() ) ) );
        // if (free_queue.size != 0) {
        //     free_queue.pop();
        //     std::cout << "QUEUE ssize: " << queue.size << std::endl;
        //     std::cout << "PIC ssize: " << pic_queue.size << std::endl;
        //     std::cout << "INFER ssize: " << infer_queue.size << std::endl;
        //     std::cout << "FREE ssize: " << free_queue.size << std::endl;
        // }
        
        

        STOP
    }
    
}

int main(int argc, char* argv[]) {

    cxxopts::Options options("app", "options for app");

    options.add_options()
    ("nr,noresize", "Whether resize or not")
    ("nds,nodisplay", "Whether display or not")
    ("ndt,nodetect", "Whether detect or not")
    ("iw,input-width", "Input width", cxxopts::value<int>()->default_value("640"))
    ("ih,input-height", "Input height", cxxopts::value<int>()->default_value("640"))
    ("dw,display-width", "Display width", cxxopts::value<int>()->default_value("640"))
    ("dh,display-height", "Display height", cxxopts::value<int>()->default_value("640"))
    ("p,model-path", "Path to .hef model", cxxopts::value<std::string>())
    ("f,frame-rate", "Input frame-rate", cxxopts::value<int>()->default_value("30"));

    result = options.parse(argc, argv);


    
    int drm_fd = open("/dev/dri/card0", O_RDWR | O_CLOEXEC);
    drm_init(drm_fd);



    // Relay relay("/dev/ttyUSB0");
    // gpio.init(39);
    signal(SIGINT, sigint_handler);
    AICamera cam;
    int w = result["input-width"].as<int>();
    int h = result["input-height"].as<int>();
    cam.setHeight(w);
    cam.setWidth(h);
    if (result.count("noresize") == 0) {
        WIDTH = result["display-width"].as<int>();
        HEIGHT = result["display-height"].as<int>();
    } else {
        WIDTH = w;
        HEIGHT = h;
    }
    
    FPS = (double)result["frame-rate"].as<int>();
    cam.setOffsetX( (2592 - w) / 2 );
    cam.setOffsetY( (1944 - h) / 2 );

    if (result.count("nodetect") == 0) {
        HailoNN* hailo = new HailoNN(result["model-path"].as<std::string>());
        NN = hailo;
    }
    

    auto start = std::chrono::high_resolution_clock::now();
    std::thread capture_thread(start_capture, std::ref(cam));
    std::thread convert_thread(start_convert);
    std::thread draw_thread;
    if (result.count("nodisplay") == 0) { 
        draw_thread = std::thread(start_draw);
    }
    std::thread inference_thread;
    if (result.count("nodetect") == 0) {
        inference_thread = std::thread(start_inference);
    }
   


    capture_thread.join();
    convert_thread.join();

    if (result.count("nodisplay") == 0) { 
        draw_thread.join();
    }
    
    if (result.count("nodetect") == 0) {
        inference_thread.join();
    }
    
    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> dura = end - start;

    std::cout << "AVERAGE FPS = " << count / (dura.count() / 1000) << std::endl;
    cam.destroy();

    drm_destroy(drm_fd);

    return 0;
}

