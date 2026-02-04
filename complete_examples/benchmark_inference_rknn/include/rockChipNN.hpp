#include "rknn_api.h"
#include "utils.hpp"
//Example for RKNN

typedef struct {

    void* input_buffer;
    int fd;
    void* output_buffer;

} standart_inference_ctx;

class RockChipNN {

    public:

        rknn_context ctx;

        rknn_input_output_num io_num;

        rknn_tensor_attr* input_attrs;

        rknn_tensor_attr* output_attrs;

        std::vector<double> times;
        int frames_processed;

        int model_in_height;

        int model_in_width;

        int req_channel;

        int wstride;

        int hstride;

        RockChipNN();

        RockChipNN(std::string path);

        void init(std::string path);

        void inference(standart_inference_ctx* ctx);

        void inference_dmabuf(standart_inference_ctx* ctx);


};