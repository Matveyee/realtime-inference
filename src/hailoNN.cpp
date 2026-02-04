#include "../include/hailoNN.hpp"
#include "../include/hailoPostprocess.hpp"

HailoNN::HailoNN() {}

HailoNN::HailoNN(std::string path) {

    init(path);

}

void HailoNN::init(std::string path) {

    vdevice = hailort::VDevice::create().expect("Failed to create vdevice");
    infer_model = vdevice->create_infer_model(path).expect("Failed to create infer model");
    configured_infer_model = infer_model->configure().expect("Failed to configure model");

}

static std::shared_ptr<uint8_t> page_aligned_alloc(size_t size)
{
#if defined(__unix__)
    auto addr = mmap(NULL, size, PROT_WRITE | PROT_READ, MAP_ANONYMOUS | MAP_PRIVATE, -1, 0);
    if (MAP_FAILED == addr) throw std::bad_alloc();
    return std::shared_ptr<uint8_t>(reinterpret_cast<uint8_t*>(addr), [size](void *addr) { munmap(addr, size); });
#elif defined(_MSC_VER)
    auto addr = VirtualAlloc(NULL, size, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
    if (!addr) throw std::bad_alloc();
    return std::shared_ptr<uint8_t>(reinterpret_cast<uint8_t*>(addr), [](void *addr){ VirtualFree(addr, 0, MEM_RELEASE); });
#else
#pragma error("Aligned alloc not supported")
#endif
}


void HailoNN::inference(standart_inference_ctx* ctx) {
    log("Inference entered");
    auto &infer_model1 = configured_infer_model;
    auto bindings = infer_model1.create_bindings().expect("Failed to create bindings");
    
    int i = 0;
    for (const auto &input_name : infer_model->get_input_names()) {
        size_t input_frame_size = infer_model->input(input_name)->get_frame_size();
      
        bindings.input(input_name)->set_buffer(hailort::MemoryView(ctx->input_buffer, input_frame_size));
        i++;
    }

    log("Inputs set");                          

    int k = 0;
    for (const auto &output_name : infer_model->get_output_names()) {
        size_t output_frame_size = infer_model->output(output_name)->get_frame_size();
        ctx->output_buffer = malloc(output_frame_size);
        bindings.output(output_name)->set_buffer(hailort::MemoryView(ctx->output_buffer, output_frame_size));

        k++;
    }


    auto start = std::chrono::high_resolution_clock::now();
    log("Trying to inference");
    auto job = infer_model1.run_async(bindings,[&](const hailort::AsyncInferCompletionInfo & info){
        log("Inferenced");
        // free(ctx->input_buffer);
        //free(ctx->output_buffer);
        log("input free success");
        
        auto bboxes = parse_nms_data((uint8_t*)ctx->output_buffer, 80);

        free(ctx->output_buffer);
        draw_bounding_boxes(map, bboxes, 640, 640, pitch, ctx->proj);
        delete[] ctx;
   
    }).expect("Failed to start async infer job");
    

}

void HailoNN::inference_dmabuf(standart_inference_ctx* ctx) {

    auto &infer_model1 = configured_infer_model;

    // создаём bindings
    auto bindings_exp = infer_model1.create_bindings();
    if (!bindings_exp) {
        std::cerr << "Failed to create bindings: " << bindings_exp.status() << std::endl;
        return;
    }
    auto bindings = std::move(bindings_exp.value());

    // -----------------------------------------------------
    //                  В Х О Д   (DMA)
    // -----------------------------------------------------
    for (const auto &input_name : infer_model->get_input_names()) {

        // получаем входной stream
        auto input_stream_exp = bindings.input(input_name);
        if (!input_stream_exp) {
            std::cerr << "Failed to get input stream: "
                    << input_stream_exp.status() << std::endl;
            return;
        }
        auto input_stream = input_stream_exp.value();

        // узнаём размер входного кадра
        size_t input_frame_size = infer_model->input(input_name)->get_frame_size();

        // готовим dma buffer
        hailo_dma_buffer_t dma_buf{};
        dma_buf.fd   = ctx->fd;             // <<< вот он — fd от RGA
        dma_buf.size = input_frame_size;   // 640*640*3

        auto status = input_stream.set_dma_buffer(dma_buf);
        if (status != HAILO_SUCCESS) {
            std::cerr << "set_dma_buffer failed: " << status << std::endl;
            return;
        }
    }

    // -----------------------------------------------------
    //                 В Ы Х О Д Ы  (CPU buffer)
    // -----------------------------------------------------

    std::vector<std::shared_ptr<uint8_t>> output_buffers;

    for (const auto &output_name : infer_model->get_output_names()) {

        size_t output_frame_size = infer_model->output(output_name)->get_frame_size();

        // CPU (page aligned) buffer
        auto out_buf = page_aligned_alloc(output_frame_size);
        output_buffers.push_back(out_buf);

        // привязываем MemoryView к bindings
        auto output_stream = bindings.output(output_name).value();
        auto status = output_stream.set_buffer(
            hailort::MemoryView(out_buf.get(), output_frame_size)
        );

        if (status != HAILO_SUCCESS) {
            std::cerr << "failed to set output buffer: " << status << std::endl;
            return;
        }
    }

    // -----------------------------------------------------
    //                 З А П У С К   I N F E R
    // -----------------------------------------------------

    auto start = std::chrono::high_resolution_clock::now();
    log("Trying to inference");
    auto job = infer_model1.run_async(bindings,[&](const hailort::AsyncInferCompletionInfo & info){
        log("Inferenced");
        // free(ctx->input_buffer);
        //free(ctx->output_buffer);
        log("input free success");
        
        auto bboxes = parse_nms_data((uint8_t*)ctx->output_buffer, 80);

        free(ctx->output_buffer);
        draw_bounding_boxes(map, bboxes, 640, 640, pitch, ctx->proj);
        delete[] ctx;
   
    }).expect("Failed to start async infer job");


}


