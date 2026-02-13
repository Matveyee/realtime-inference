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
#include "MPPEntity.hpp"
#include "Pool.hpp"

#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define RESET   "\033[0m"
#define SIZE 640

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
MPPEntity mpp;
V4L2Camera cam_v4l2;

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

class DMABufferQueueObject {
    
    public:

        struct v4l2_buffer* buf;
        int v4l2_fd;

        DMABufferQueueObject(struct v4l2_buffer* buffer, int fd) : buf(buffer), v4l2_fd(fd) {}

        void close() {
            if (ioctl(v4l2_fd, VIDIOC_QBUF, buf) < 0) {
                perror("VIDIOC_QBUF");
            }
        }
};



Queue<FrameBufferQueueObject> queue;
Queue<BufferQueueObject> pic_queue;
Queue<BufferQueueObject> infer_queue;
Queue<DMAPoolBufferObject> pic_queue_dmabuf;
Queue<DMAPoolBufferObject> infer_queue_dmabuf;
Queue<DMABufferQueueObject> v4l2_queue;
Queue<MppFrameQueueObject> decoded_queue;
Pool<DMAPoolBufferObject, 15> nv12_pool;
Pool<DMAPoolBufferObject, 15> rgb_pool;


void start_inference() {
    while (true) {
        STOP

        if (infer_queue.size == 0) {
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
            continue;
        }
        uint8_t* data = pic_queue.read().data;
        drawPicture(data, WIDTH, HEIGHT);
        draw_index++;
        std::cout << "index " << cap_index << std::endl;
        if (cap_index % 10 == 0){ 
            std::cout << "DELAY: " << cap_index <<"; " << conv_index << "; " << draw_index <<"; "<< cap_index - draw_index << std::endl;
        }
        
        
        if (result.count("nodetect") == 0) {
            infer_queue.push(data);
            pic_queue.remove();
        } else {
            pic_queue.pop();
        }
        std::cout << "size before" << pic_queue.size << std::endl; 
        
        std::cout << "size after" << pic_queue.size << std::endl;

        STOP
    }
}



void start_convert() {
    
    while (true) {
        STOP
        if (queue.size <= 0) {
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

        queue.pop();
        
        conv_index++;

        

        STOP
    }
}





int count = 0;
void start_capture(AICamera* cam_ptr) {
    AICamera cam = *cam_ptr;
    auto start = simple_now();

    while (true) {
        STOP
        
        if (queue.size >= 4) {

            continue;

        }
        
        PGX_FRAME_BUFFER pFrameBuffer;
        
        cam.startStream();
        
        if (cam.getBuffer(&pFrameBuffer) == GX_STATUS_SUCCESS) {
            continue;
        }
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

        STOP
    }
    
}
void start_inference_dmabuf() {

    while (true) {
        STOP

        if (infer_queue_dmabuf.size == 0) {
            continue;
        }

        standart_inference_ctx* ctx = new standart_inference_ctx();
        ctx->fd = infer_queue_dmabuf.read().fd;

        context = ctx;
        NN->inference_dmabuf(context);
        infer_queue_dmabuf.pop();

    }

}

void start_draw_v4l2() {
    while (true) {
        STOP
        if (pic_queue_dmabuf.size <= 0) {
            continue;
        }
        int fd = pic_queue_dmabuf.read().fd;
        drawPicture(fd, WIDTH, HEIGHT);
        draw_index++;
        std::cout << "index " << cap_index << std::endl;
        if (cap_index % 10 == 0){ 
            std::cout << "DELAY: " << cap_index <<"; " << conv_index << "; " << draw_index <<"; "<< cap_index - draw_index << std::endl;
        }
        
        
        // if (result.count("nodetect") == 0) {
        //     infer_queue.push(data);
        //     pic_queue.remove();
        // } else {
        pic_queue_dmabuf.pop();
        // }
        // std::cout << "size before" << pic_queue.size << std::endl; 
        
        // std::cout << "size after" << pic_queue.size << std::endl;

        STOP
    }
}

void start_convert_v4l2() {
    
    nv12_pool.init(SIZE*SIZE*3/2);
    rgb_pool.init(SIZE*SIZE*3);
    while (true) {
        
        STOP
        if (decoded_queue.size == 0) {
            continue;
        }
        auto mfqo = decoded_queue.read();
        MppFrame frame = mfqo.frame;

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
        pic_queue_dmabuf.push(rgb_buffer);
        

    }
    
}

void start_decode() {

    while (true) {
        
        STOP
        if (v4l2_queue.size == 0) {
            continue;
        }
        DMABufferQueueObject dbqo = v4l2_queue.read();
        void *src = mmap(nullptr, dbqo.buf->bytesused, PROT_READ, MAP_SHARED, cam_v4l2.buffers[dbqo.buf->index], 0);
        int ret = mpp.put_packet(src, dbqo.buf->length);
        if (ret) {
            error("put_packet_dma error: ", ret);
            stop = 1;
            break;
        }

        v4l2_queue.pop();

        MppFrame frame;
        ret = mpp.try_get_frame(&frame);

        if (ret) {
            error("mpp.try_get_frame: ", ret);
            stop = 1;
            break;
        } else {
            std::cout << "returned 0" << std::endl;
            continue;
        }

        decoded_queue.push(frame);

        
        
    }
    

}


void start_capture_v4l2() {

   
    cam_v4l2.startCapture();
    auto start = simple_now();

    while (true) {
        STOP

        struct v4l2_buffer* buf = new struct v4l2_buffer();
        
        memset(buf, 0, sizeof(struct v4l2_buffer));
        buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf->memory = V4L2_MEMORY_MMAP;

        if (ioctl(cam_v4l2.fd, VIDIOC_DQBUF, buf) < 0) {
            perror("VIDIOC_DQBUF");
        }

    

        DMABufferQueueObject dbqo(buf, cam_v4l2.fd);
        v4l2_queue.push(dbqo);

        
    }

}

void start_test( ) {
    cam_v4l2.startCapture();
    while (!stop) {

    // 1. Вытягиваем один буфер из V4L2
    struct v4l2_buffer v4l2buf = {};
    v4l2buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(cam_v4l2.fd, VIDIOC_DQBUF, &v4l2buf) < 0) {
        perror("VIDIOC_DQBUF");
        continue;
    }

    int index    = v4l2buf.index;
    int dma_fd   = cam_v4l2.buffers[index];
    int used     = v4l2buf.bytesused;
    int total    = cam_v4l2.sizes[index];

    void* src = mmap(nullptr, used, PROT_READ, MAP_SHARED, dma_fd, 0);
    void* dst = malloc(used);
    memcpy(dst, src, used);
   
    munmap(src, used);
    // stop = 1;
    // break;
    std::cout << "frame: index=" << index
              << " fd=" << dma_fd
              << " used=" << used
              << " total=" << total << std::endl;

    // 2. Кладём пакет в декодер
    int pret = mpp.put_packet(dst, used);
    if (pret < 0) {
        std::cout << "put_packet_dma error" << std::endl;
        break;
    }

    // 3. Сразу возвращаем буфер камере
    if (ioctl(cam_v4l2.fd, VIDIOC_QBUF, &v4l2buf) < 0) {
        perror("VIDIOC_QBUF");
    }

    // 4. Пытаемся вытащить ВСЕ готовые кадры
    while (!stop) {
        MppFrame frame = nullptr;
        MPP_RET ret = mpp.mpi->decode_get_frame(mpp.ctx, &frame);

        if (ret != MPP_OK) {
            // реальная ошибка
            std::cout << "decode_get_frame ret=" << ret << std::endl;
            break;
        }

        if (!frame) {
            // на сейчас нет кадров – выходим из внутреннего цикла
            break;
        }

        if (mpp_frame_get_info_change(frame)) {
            mpp.handle_info_change(mpp.ctx, mpp.mpi, frame, mpp.ext_group, mpp.ext_group_inited);
            mpp_frame_deinit(&frame);
            continue;
        }

        if (mpp_frame_get_errinfo(frame)) {
            // битый кадр
            mpp_frame_deinit(&frame);
            continue;
        }

        // Тут у тебя валидный декодированный кадр
        std::cout << "got decoded frame " << frame << std::endl;

        // На первом шаге – НИЧЕГО с ним не делаем, только освобождаем:
        mpp_frame_deinit(&frame);
    }
}

}

void start_pipeline_for_v4l2_camera() {

    int drm_fd = open("/dev/dri/card0", O_RDWR | O_CLOEXEC);
    drm_init();

    signal(SIGINT, sigint_handler);

    

    int w = result["input-width"].as<int>();
    int h = result["input-height"].as<int>();
    if (result.count("noresize") == 0) {
        WIDTH = result["display-width"].as<int>();
        HEIGHT = result["display-height"].as<int>();
    } else {
        WIDTH = w;
        HEIGHT = h;
    }
    FPS = (double)result["frame-rate"].as<int>();


    cam_v4l2.init(result["camera-path"].as<std::string>());

    
    cam_v4l2.setHeight(h);
    cam_v4l2.setWidth(w);

    mpp.init();

    if (result.count("nodetect") == 0) {
        HailoNN* hailo = new HailoNN(result["model-path"].as<std::string>());
        NN = hailo;
    }
    std::thread test(start_test);
    test.join();
    // auto start = std::chrono::high_resolution_clock::now();
    // std::thread capture_thread(start_capture_v4l2);
    // std::thread decode_thread(start_decode);
    // std::thread convert_thread(start_convert_v4l2);
    


    // std::thread draw_thread;
    // if (result.count("nodisplay") == 0) { 
    //     draw_thread = std::thread(start_draw);
    // }
    // std::thread inference_thread;
    // if (result.count("nodetect") == 0) {
    //     inference_thread = std::thread(start_inference_dmabuf);
    // }


    // capture_thread.join();
    // decode_thread.join();
    // convert_thread.join();

    // if (result.count("nodisplay") == 0) { 
    //     draw_thread.join();
    // }
    
    // if (result.count("nodetect") == 0) {
    //     inference_thread.join();
    // }
    
    // auto end = std::chrono::high_resolution_clock::now();

    // std::chrono::duration<double, std::milli> dura = end - start;

    // std::cout << "AVERAGE FPS = " << count / (dura.count() / 1000) << std::endl;
    cam_v4l2.destroy();

    drm_destroy();
}

void start_pipeline_for_gx_camera() {

    
    drm_init();

    signal(SIGINT, sigint_handler);

    AICamera cam;

    int w = result["input-width"].as<int>();
    int h = result["input-height"].as<int>();
    if (result.count("noresize") == 0) {
        WIDTH = result["display-width"].as<int>();
        HEIGHT = result["display-height"].as<int>();
    } else {
        WIDTH = w;
        HEIGHT = h;
    }
    FPS = (double)result["frame-rate"].as<int>();


    cam.init();
    
    cam.setHeight(h);
    cam.setWidth(w);
    
    
    
    cam.setOffsetX( (2592 - w) / 2 );
    cam.setOffsetY( (1944 - h) / 2 );

    if (result.count("nodetect") == 0) {
        HailoNN* hailo = new HailoNN(result["model-path"].as<std::string>());
        NN = hailo;
    }
    

    auto start = std::chrono::high_resolution_clock::now();
    std::thread capture_thread;
    std::thread convert_thread;

    capture_thread = std::thread(start_capture, &cam);
    convert_thread = std::thread(start_convert);

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

    drm_destroy();

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
    ("cp,camera-path", "Path to v4l2 (/dev/video*) camera", cxxopts::value<std::string>())
    ("f,frame-rate", "Input frame-rate", cxxopts::value<int>()->default_value("30"));

    result = options.parse(argc, argv);

    if (result.count("camera-path") == 0) {
        start_pipeline_for_gx_camera();
    } else {
        start_pipeline_for_v4l2_camera();
    }
    
    return 0;
}

