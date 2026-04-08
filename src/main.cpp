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
#include "libyuv.h"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define RESET   "\033[0m"
#define SIZE 640

int w;
int h;
int WIDTH;
int HEIGHT;
int FPS;
int sizee;
int COUNT;
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
DrmContext drm;


static const char *mpp_fmt_to_str(MppFrameFormat fmt_id)
{
    switch (fmt_id) {
    case MPP_FMT_YUV420SP:     return "YUV420SP (NV12)";
    case MPP_FMT_YUV420SP_VU:  return "YUV420SP_VU (NV21)";
    case MPP_FMT_YUV420P:      return "YUV420P (I420)";
    case MPP_FMT_YUV422SP:     return "YUV422SP (NV16)";
    case MPP_FMT_YUV422P:      return "YUV422P";
    default:                   return "OTHER";
    }
}

static void dump_mpp_frame_to_file(MppFrame frame, FILE *fp)
{
    if (!frame || !fp)
        return;

    MppBuffer buffer = mpp_frame_get_buffer(frame);
    if (!buffer)
        return;

    int width    = mpp_frame_get_width(frame);
    int height   = mpp_frame_get_height(frame);
    int hstride  = mpp_frame_get_hor_stride(frame);
    int vstride  = mpp_frame_get_ver_stride(frame);
    size_t bufsz = mpp_buffer_get_size(buffer);

    MppFrameFormat fmt_full = static_cast<MppFrameFormat>(mpp_frame_get_fmt(frame));
    MppFrameFormat fmt_id = static_cast<MppFrameFormat>(fmt_full & MPP_FRAME_FMT_MASK);

    uint8_t *base = (uint8_t *)mpp_buffer_get_ptr(buffer);
    if (!base)
        return;

    fprintf(stderr,
            "Dump frame: %dx%d, hstride=%d, vstride=%d, fmt_full=%d, fmt_id=%d (%s), buf=%zu\n",
            width, height, hstride, vstride,
            fmt_full, fmt_id, mpp_fmt_to_str(fmt_id), bufsz);

    switch (fmt_id) {
    case MPP_FMT_YUV420SP:          // NV12
    case MPP_FMT_YUV420SP_VU:      // NV21 (раскладка та же, только порядок U/V другой)
    {
        uint8_t *y_plane  = base;
        uint8_t *uv_plane = base + hstride * vstride;

        // Y: height строк по width байт
        for (int y = 0; y < height; y++) {
            fwrite(y_plane + y * hstride, 1, width, fp);
        }

        // UV/VU: height/2 строк по width байт
        for (int y = 0; y < height / 2; y++) {
            fwrite(uv_plane + y * hstride, 1, width, fp);
        }

        break;
    }

    case MPP_FMT_YUV420P:          // планарный 4:2:0
    {
        uint8_t *y_plane = base;
        uint8_t *u_plane = y_plane + hstride * vstride;
        uint8_t *v_plane = u_plane + (hstride / 2) * (vstride / 2);

        // Y
        for (int y = 0; y < height; y++) {
            fwrite(y_plane + y * hstride, 1, width, fp);
        }
        // U
        for (int y = 0; y < height / 2; y++) {
            fwrite(u_plane + y * (hstride / 2), 1, width / 2, fp);
        }
        // V
        for (int y = 0; y < height / 2; y++) {
            fwrite(v_plane + y * (hstride / 2), 1, width / 2, fp);
        }

        break;
    }

    case MPP_FMT_YUV422SP:         // NV16: Y full + interleaved UV full
    {
        uint8_t *y_plane  = base;
        uint8_t *uv_plane = base + hstride * vstride;

        // Y: height строк по width байт
        for (int y = 0; y < height; y++) {
            fwrite(y_plane + y * hstride, 1, width, fp);
        }

        // UV: height строк по width байт (4:2:2 → вертикального подвыборa нет)
        for (int y = 0; y < height; y++) {
            fwrite(uv_plane + y * hstride, 1, width, fp);
        }

        break;
    }

    default:
    {
        // на крайний случай – выводим предупреждение и НЕ дампим весь буфер,
        // чтобы не получать странные размеры
        fprintf(stderr, "Unsupported format %d, skip dump\n", fmt_id);
        break;
    }
    }
}


int append_mpp_frame_as_nv12(FILE *fp, MppFrame frame) {
    if (!fp || !frame) {
        std::cerr << "append_mpp_frame_as_nv12: invalid argument\n";
        return -1;
    }

    MppBuffer buffer = mpp_frame_get_buffer(frame);
    if (!buffer) {
        std::cerr << "append_mpp_frame_as_nv12: frame has no buffer\n";
        return -1;
    }

    void *ptr = mpp_buffer_get_ptr(buffer);
    if (!ptr) {
        std::cerr << "append_mpp_frame_as_nv12: buffer ptr is null\n";
        return -1;
    }

    const RK_U32 width      = mpp_frame_get_width(frame);
    const RK_U32 height     = mpp_frame_get_height(frame);
    const RK_U32 hor_stride = mpp_frame_get_hor_stride(frame);
    const RK_U32 ver_stride = mpp_frame_get_ver_stride(frame);
    const RK_U32 fmt        = mpp_frame_get_fmt(frame);

    const uint8_t *base = static_cast<const uint8_t *>(ptr);

    // ---- Layout assumptions for semi-planar formats ----
    // Y plane:  hor_stride * ver_stride
    // UV plane starts right after Y plane
    const uint8_t *src_y  = base;
    const uint8_t *src_uv = base + hor_stride * ver_stride;

    // 1) Write Y plane to output as tightly packed width x height
    for (RK_U32 y = 0; y < height; ++y) {
        const uint8_t *row = src_y + y * hor_stride;
        if (fwrite(row, 1, width, fp) != width) {
            std::cerr << "append_mpp_frame_as_nv12: failed to write Y plane\n";
            return -1;
        }
    }

    // 2) Write UV plane in NV12
    // Supported inputs:
    // - MPP_FMT_YUV420SP  (already NV12-like layout)
    // - MPP_FMT_YUV422SP  (NV16 -> convert to NV12)
    if (fmt == MPP_FMT_YUV420SP) {
        // Already UV interleaved with half vertical resolution
        for (RK_U32 y = 0; y < height / 2; ++y) {
            const uint8_t *row = src_uv + y * hor_stride;
            if (fwrite(row, 1, width, fp) != width) {
                std::cerr << "append_mpp_frame_as_nv12: failed to write UV plane\n";
                return -1;
            }
        }
    } else if (fmt == MPP_FMT_YUV422SP) {
        // NV16 -> NV12
        // In NV16, UV is interleaved and has full vertical resolution.
        // For NV12, UV must have half vertical resolution.
        // We'll average every two UV rows.
        std::vector<uint8_t> uv_row(width);

        for (RK_U32 y = 0; y < height / 2; ++y) {
            const uint8_t *row0 = src_uv + (2 * y + 0) * hor_stride;
            const uint8_t *row1 = src_uv + (2 * y + 1) * hor_stride;

            for (RK_U32 x = 0; x < width; ++x) {
                uv_row[x] = static_cast<uint8_t>(
                    (static_cast<unsigned>(row0[x]) + static_cast<unsigned>(row1[x])) / 2
                );
            }

            if (fwrite(uv_row.data(), 1, width, fp) != width) {
                std::cerr << "append_mpp_frame_as_nv12: failed to write converted UV plane\n";
                return -1;
            }
        }
    } else {
        std::cerr << "append_mpp_frame_as_nv12: unsupported MPP format: " << fmt << "\n";
        return -1;
    }

    return 0;
}

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
        int real_fd;

        DMABufferQueueObject(struct v4l2_buffer* buffer, int fd, int real) : buf(buffer), v4l2_fd(fd), real_fd(real) {}

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

void destroy_application() {
    
    cam_v4l2.destroy();
    mpp.destroy();
    // drm_destroy();

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
        // uint8_t* img = (uint8_t*)mmap(NULL, 640*640*3, PROT_READ, MAP_SHARED, fd, 0);
        // drawPicture(img, 640, 640);
        // munmap(img, 640*640*3);
        uint32_t pitches[4] = {
            640 * 3,   // 3 байта на пиксель
            0, 0, 0
        };
        uint32_t offsets[4] = {
            0,
            0, 0, 0
        };

        drm_show_dmabuf(
            drm,
            fd,
            640,
            640,
            DRM_FORMAT_BGR888,
            pitches,
            offsets
        );
        draw_index++;

        pic_queue_dmabuf.pop();

        STOP
    }
}

void start_convert_v4l2() {
    std::cout << "convert entered" << std::endl;
    
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

        std::cout << "resized" << std::endl;
        
        decoded_queue.pop();

        DMAPoolBufferObject rgb_buffer = rgb_pool.capture();

        final_dst = wrapbuffer_fd(rgb_buffer.fd, SIZE, SIZE, RK_FORMAT_RGB_888);

        status = imcvtcolor(dst, final_dst, RK_FORMAT_YCbCr_420_SP, RK_FORMAT_RGB_888);
        
        std::cout << "converted" << std::endl;

        dst_buffer.release();
        pic_queue_dmabuf.push(rgb_buffer);
        
    }
    
}

void start_decode() {

    // FILE *nv12_file = fopen("video_nv12.yuv", "wb");
    // if (!nv12_file) {
    //     perror("fopen video_nv12.yuv");
    //     return;
    // }

    while (true) {
        
        STOP
        if (v4l2_queue.size <= 2) {
            continue;
        }
        DMABufferQueueObject dbqo = v4l2_queue.read();
        MppFrame frame = NULL;
      
        int ret = mpp.mjpeg_decode(cam_v4l2.pointers[  dbqo.buf->index ], dbqo.buf->bytesused, &frame, w, h);
        // std::cout << "DECODED" << std::endl;
        // if (ret == -1) {
        //     stop = 1;
        //     break;
        // }

        v4l2_queue.pop();

        
        // if (append_mpp_frame_as_nv12(nv12_file, frame) != 0) {
        //     std::cerr << "failed to append frame to nv12 file\n";
        // }

        
        
        decoded_queue.push(frame);
        // decoded_queue.pop();
        
    }
    // fclose(nv12_file);

}


void start_capture_v4l2() {

   
    cam_v4l2.startCapture();
    auto start = simple_now();
    int i = 0;

    while (true) {
        STOP
        if (COUNT != 0) {
            if (i == COUNT) {
                stop = 1;
                break;
            }
        }
        if (v4l2_queue.size >= 4) {
            continue;
        }
        struct v4l2_buffer* v4l2buf = new struct v4l2_buffer();

        v4l2buf->type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2buf->memory = V4L2_MEMORY_MMAP;


        if (ioctl(cam_v4l2.fd, VIDIOC_DQBUF, v4l2buf) < 0) {
            perror("VIDIOC_DQBUF");
        }


        int index    = v4l2buf->index;
        int dma_fd   = cam_v4l2.buffers[index];
        int used     = v4l2buf->bytesused;
        int total    = cam_v4l2.sizes[index];
    
        auto end = simple_now();
        Duration dur = end - start;
        start = end;
        std::this_thread::sleep_for(std::chrono::milliseconds( (int)( 1000.0 / FPS - dur.count())));
        DMABufferQueueObject dbqo(v4l2buf, cam_v4l2.fd, dma_fd);
        v4l2_queue.push(dbqo);

        i++;
    }

}

void start_test( ) {


    FILE *nv12_file = fopen("video_nv12.yuv", "wb");
    if (!nv12_file) {
        perror("fopen video_nv12.yuv");
        return;
    }
    
    cam_v4l2.startCapture();
    auto start = simple_now();
    int i = 0;
    while (true) {
        STOP
        std::cout << i <<std::endl;
        if (COUNT != 0) {
            if (i == COUNT) {
                stop = 1;
                break;
            }
        }

        struct v4l2_buffer* v4l2buf = new struct v4l2_buffer();

        v4l2buf->type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2buf->memory = V4L2_MEMORY_MMAP;


        if (ioctl(cam_v4l2.fd, VIDIOC_DQBUF, v4l2buf) < 0) {
            perror("VIDIOC_DQBUF");
        }


        int index    = v4l2buf->index;
        int dma_fd   = cam_v4l2.buffers[index];
        void* ptr = cam_v4l2.pointers[index];
        int used     = v4l2buf->bytesused;
        int total    = cam_v4l2.sizes[index];

        auto end = simple_now();
        Duration dur = end - start;
        start = end;
        std::this_thread::sleep_for(std::chrono::milliseconds( (int)( 1000.0 / FPS - dur.count())));

        MppFrame frame = NULL;
      
        int ret = mpp.mjpeg_decode(ptr, used, &frame, w, h);
        
        if (ioctl(cam_v4l2.fd, VIDIOC_QBUF, v4l2buf) < 0) {
            perror("VIDIOC_QBUF");
        }

        delete[] v4l2buf;

        if (ret == -1) {
            stop = 1;
            MppBuffer frm_buf = mpp_frame_get_buffer(frame);
            mpp_buffer_put(frm_buf);
            mpp_frame_deinit(&frame);
           
            break;
        }


        if (append_mpp_frame_as_nv12(nv12_file, frame) != 0) {
            std::cerr << "failed to append frame to nv12 file\n";
        }

        MppBuffer frm_buf = mpp_frame_get_buffer(frame);
        mpp_buffer_put(frm_buf);
        mpp_frame_deinit(&frame);
        
        i++;
   
    }
    destroy_application();
    fclose(nv12_file);

}

void start_pipeline_for_v4l2_camera() {

    
    drm_init(drm);
    

    signal(SIGINT, sigint_handler);

    

    w = result["input-width"].as<int>();
    h = result["input-height"].as<int>();
    if (result.count("noresize") == 0) {
        WIDTH = result["display-width"].as<int>();
        HEIGHT = result["display-height"].as<int>();
    } else {
        WIDTH = w;
        HEIGHT = h;
    }
    FPS = (int)result["frame-rate"].as<int>();
    COUNT = (int)result["count"].as<int>();

    cam_v4l2.init(result["camera-path"].as<std::string>());

    
    cam_v4l2.setResolution(w, h);

    mpp.init();
    mpp.mjpeg_mode(w, h);

    if (result.count("nodetect") == 0) {
        HailoNN* hailo = new HailoNN(result["model-path"].as<std::string>());
        NN = hailo;
    }
    if (result.count("test") == 1) { 
        std::thread test(start_test);
        test.join();
        cam_v4l2.destroy();
        mpp.destroy();
        // drm_destroy();
        return;
    } else { 
    
        auto start = std::chrono::high_resolution_clock::now();
        std::thread capture_thread(start_capture_v4l2);

        std::thread decode_thread;
        if (result.count("nodecode") == 0) { 
            decode_thread = std::thread(start_decode);
        }

        std::thread convert_thread;
        if (result.count("noconvert") == 0) { 
            convert_thread = std::thread(start_convert_v4l2);
        }

        std::thread draw_thread;
        if (result.count("nodisplay") == 0) { 
            draw_thread = std::thread(start_draw_v4l2);
        }
        std::thread inference_thread;
        if (result.count("nodetect") == 0) {
            inference_thread = std::thread(start_inference_dmabuf);
        }

        capture_thread.join();
        
        if (result.count("nodecode") == 0) { 
            decode_thread.join();
        }

        if (result.count("noconvert") == 0) { 
            convert_thread.join();
        }

        if (result.count("nodisplay") == 0) { 
            draw_thread.join();
        }
        
        if (result.count("nodetect") == 0) {
            inference_thread.join();
        }
        
        auto end = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double, std::milli> dura = end - start;

        std::cout << "AVERAGE FPS = " << count / (dura.count() / 1000) << std::endl;
        cam_v4l2.destroy();
        drm_destroy();
        mpp.destroy();
    }
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
    ("ncv,noconvert", "Whether convert or not")
    ("ndc,nodecode", "Whether decode or not")
    ("t,test", "Whether to run a test pipeline or not")
    ("iw,input-width", "Input width", cxxopts::value<int>()->default_value("640"))
    ("ih,input-height", "Input height", cxxopts::value<int>()->default_value("640"))
    ("dw,display-width", "Display width", cxxopts::value<int>()->default_value("640"))
    ("dh,display-height", "Display height", cxxopts::value<int>()->default_value("640"))
    ("p,model-path", "Path to .hef model", cxxopts::value<std::string>())
    ("cp,camera-path", "Path to v4l2 (/dev/video*) camera", cxxopts::value<std::string>())
    ("f,frame-rate", "Input frame-rate", cxxopts::value<int>()->default_value("30"))
    ("c,count", "How many frames to capture", cxxopts::value<int>()->default_value("0"));

    result = options.parse(argc, argv);

    if (result.count("camera-path") == 0) {
        start_pipeline_for_gx_camera();
    } else {
        start_pipeline_for_v4l2_camera();
    }
    
    return 0;
}

