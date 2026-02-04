#include "hailo/hailort.hpp"
#include "utils.hpp"

typedef struct {

    void* input_buffer;
    int fd;
    void* output_buffer;

} standart_inference_ctx;

class HailoNN {

    public:

        std::unique_ptr<hailort::VDevice> vdevice;
        std::shared_ptr<hailort::InferModel> infer_model;
        hailort::ConfiguredInferModel configured_infer_model;
        std::vector<double> times;
        int frames_processed;
        bool show_result;

        HailoNN();

        HailoNN(std::string path);

        void init(std::string path);

        void inference(standart_inference_ctx* ctx);

        void inference_dmabuf(standart_inference_ctx* ctx);

        void do_show_result();

};

struct NamedBbox {
    hailo_bbox_float32_t bbox;
    size_t class_id;
};

std::vector<NamedBbox> parse_nms_data(uint8_t* data, size_t max_class_count);

void draw_bounding_boxes(uint8_t* map, const std::vector<NamedBbox>& bboxes, int width, int height, uint32_t pitch);