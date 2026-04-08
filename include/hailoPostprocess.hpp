#include "hailo/hailort.hpp"
#include "../include/utils.hpp"


std::vector<NamedBbox> parse_nms_data(uint8_t* data, size_t max_class_count);

void draw_bounding_boxes(uint8_t* map, const std::vector<NamedBbox>& bboxes, int width, int height, uint32_t pitch, Projection proj);