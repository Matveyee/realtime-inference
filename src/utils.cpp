#include "../include/utils.hpp"
#include <cstring>

#include <algorithm>


void log(std::string message) {
    if (DEBUG)
        std::cout << "DEBUG: " << message << std::endl;
}
void error(std::string err, int code) {
    
    std::cout << RED << "ERROR: " << err << code << RESET << std::endl;

}

uint16_t modbus_crc16( const unsigned char *buf, unsigned int len ) {
	uint16_t crc = 0xFFFF;
	unsigned int i = 0;
	char bit = 0;

	for( i = 0; i < len; i++ )
	{
		crc ^= buf[i];

		for( bit = 0; bit < 8; bit++ )
		{
			if( crc & 0x0001 )
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
			{
				crc >>= 1;
			}
		}
	}
    return crc;
}

drmModeRes* res;
drmModeConnector* conn;
uint32_t conn_id;
drmModeCrtc *old_crtc = nullptr;
uint8_t* map = nullptr;
uint32_t fb_id;
uint32_t handle;
uint32_t pitch;
uint64_t size;
uint32_t crtc_id;
drmModeModeInfo mode;
int drm_fd;
struct drm_mode_fb_cmd fb = {};
DrmContext drm;



Vec::Vec(int p_x, int p_y) : x(p_x), y(p_y) {}
Vec::Vec() {}

void Vec::init(int p_x, int p_y) {
    x = p_x;
    y = p_y;
}

int Vec::getx() {
    return x;
}

int Vec::gety() {
    return y;
}
Vec& Vec::operator += (Vec& other) {
    x += other.getx();
    y += other.gety();
    return *this;
}
void Vec::print() {
    std::cout << "( " << x << "," << y << ")";
}



Vec operator +(Vec& first, Vec& second) {

    Vec vec(first.getx() + second.getx(), first.gety() + second.gety());
    return vec;

}

Projection::Projection() {}

Projection::Projection(uint8_t* data_ptr, int offset_x, int offset_y, int source_w, int source_h, int width, int height) : data(data_ptr), offsetX(offset_x), offsetY(offset_y), w(width), h(height), sourceW(source_w), sourceH(source_h) {}

uint8_t Projection::operator [](int index) {   
    
    return data[(offsetY + index / w) * sourceW + offsetX + index % w];
}
void Projection::init(uint8_t* data_ptr , int offset_x, int offset_y, int source_w, int source_h, int width, int height) {
    data = data_ptr;
    offsetX = offset_x;
    offsetY = offset_y;
    w = width;
    h = height;
    sourceW = source_w;
    sourceH = source_h;
}
uint8_t Projection::get(int x, int y) {
    if (x > w || y > h) {
        return 0;
    } else {
        return (*this)[x + y * w];
    }
    
}

uint8_t Projection::get(Vec vec) {
    return get(vec.getx(), vec.gety());
}

int Projection::getx() {
    return offsetX;
}

int Projection::gety() {
    return offsetY;
}

int Projection::getW() {
    return w;
}

int Projection::getH() {
    return h;
}


void draw_line(uint8_t* map, int x, int y, int x1, int y1, uint32_t pitch) {
    uint32_t pixel = (0xFF << 24) | (255 << 16) | (0 << 8) | 0;
    if (x == x1) {
        for (int i = y; i <= y1; i++) {
            ((uint32_t*)(map + i * pitch))[x] = pixel;
        }
    } else if (y == y1) {
        for (int i = x; i <= x1; i++) {
            ((uint32_t*)(map + y * pitch))[i] = pixel;
        }
    }
    
}

void draw_rect(uint8_t* map, int x, int y, int x1, int y1, uint32_t pitch) {
    draw_line(map,x,y,x1,y, pitch);
    draw_line(map,x,y1,x1,y1, pitch);
    draw_line(map,x,y,x,y1, pitch);
    draw_line(map,x1,y,x1,y1, pitch);
}






void specifyVectors(Vec& R, Vec& G1, Vec& G2, Vec& B, Vec r) {
    
    if ( (r.getx() % 2 == 0) && (r.gety() % 2 == 0)) {

        R.init(0,0);
        G1.init(1,0);
        G2.init(0,1);
        B.init(1,1);
    }else if ((r.getx() % 2 != 0) && (r.gety() % 2 == 0)) {

        G1.init(0,0);
        R.init(0,1);
        B.init(1,0);
        G2.init(1,1);
    }else if ((r.getx() % 2 == 0) && (r.gety() % 2 != 0)) {

        G2.init(0,0);
        B.init(1,0);
        R.init(0,1);
        G1.init(1,1);
    } else {

        B.init(0,0);
        G2.init(1,0);
        G1.init(0,1);
        R.init(1,1);
    }


}
uint8_t* toRGB(Projection input) {
//    debug("to rgb entered");
   // debug("entered to rgb");
    uint8_t* rgbData = (uint8_t*)calloc(  input.getW() * input.getH() * 3, sizeof(uint8_t));
   // debug("mem allocated");
    Vec r;
    Vec r0;
    Vec R;
    Vec G1;
    Vec G2;
    Vec B;
    r.init(0,0);
    r0.init(input.getx(), input.gety());
   // debug("before cycle");
    for (int i = 0; i < (input.getW() * input.getH()); i++ ) {
        // if (i % 10 == 0) {
        //     std::cout << i << std::endl;
        // }
        Vec d = r0 + r;
        specifyVectors(R, G1, G2, B, d);
        rgbData[i * 3 + 0] = input.get(r + R);
        rgbData[i * 3 + 1] = ( input.get(r + G1) + input.get(r + G2) ) / 2;
        rgbData[i * 3 + 2] = input.get(r + B);
        

        Vec delta;
        if ( (i + 1) % input.getW() == 0 ) {
            delta.init(1 - input.getW(), 1);
        } else {
            delta.init(1,0);
        }
        r += delta;
    }
   // debug("before return");
   return rgbData;
}



void drawPicture(Projection* proj, uint8_t* data) {
    log("Draw picture entered");
     for (int y = 0; y < 640; ++y) {
        for (int x = 0; x < 640; ++x) {
            int src_offset = y * 640 * 3  + x * 3;
            
            uint8_t r = data[src_offset + 0];
            uint8_t g = data[src_offset + 1];
            uint8_t b = data[src_offset + 2];

            uint32_t pixel = (0xFF << 24) | (r << 16) | (g << 8) | b;
          
            ((uint32_t*)(map + (y) * pitch))[x] = pixel;
        }

    }
    log("drawPicture exit");
}

void drawPicture(uint8_t* data,int w, int h) {
    log("Draw picture entered");
     for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int src_offset = y * w * 3  + x * 3;
            
            uint8_t r = data[src_offset + 0];
            uint8_t g = data[src_offset + 1];
            uint8_t b = data[src_offset + 2];

            uint32_t pixel = (0xFF << 24) | (r << 16) | (g << 8) | b;
          
            ((uint32_t*)(map + (y) * pitch))[x] = pixel;
        }

    }
    log("drawPicture exit");
}

void drawPicture(int fd, int w, int h) {
    uint32_t handle;
    drmPrimeFDToHandle(drm_fd, fd, &handle);

    uint32_t handles[4] = { handle, 0, 0, 0 };
    uint32_t pitches[4] = { 0, 0, 0, 0 }; // stride в байтах
    uint32_t offsets[4] = { 0, 0, 0, 0 };

    uint32_t fb_id;
    int ret = drmModeAddFB2(drm_fd, w, h,
                DRM_FORMAT_RGB888,
                handles, pitches, offsets,
                &fb_id, 0);

    if (ret) {
        perror("drmModeAddFB2");
        // не забыть закрыть handle
        struct drm_gem_close arg = { .handle = handle };
        drmIoctl(drm_fd, DRM_IOCTL_GEM_CLOSE, &arg);
        return;
    }

    // 3. Подключаем этот framebuffer к CRTC → КАРТИНКА ПОЯВЛЯЕТСЯ
    ret = drmModeSetCrtc(drm_fd,
                         crtc_id,
                         fb_id,
                         0, 0,                // x, y на экране
                         &conn_id, 1,         // коннекторы
                         &mode);              // режим (разрешение/частота)
    if (ret) {
        perror("drmModeSetCrtc");
        drmModeRmFB(drm_fd, fb_id);
        struct drm_gem_close arg = { .handle = handle };
        drmIoctl(drm_fd, DRM_IOCTL_GEM_CLOSE, &arg);
        return;
    }

    // 4. (Опционально) почистить предыдущий fb/handle, чтобы не было утечек
    // if (current_fb_id) {
    //     drmModeRmFB(drm_fd, current_fb_id);
    // }
    // if (current_handle) {
    //     struct drm_gem_close arg = { .handle = current_handle };
    //     drmIoctl(drm_fd, DRM_IOCTL_GEM_CLOSE, &arg);
    // }

    // current_fb_id = fb_id;
    // current_handle = handle;
}

uint32_t find_plane_for_format(int drm_fd,
                                      drmModeRes* res,
                                      uint32_t crtc_id,
                                      uint32_t fourcc,
                                      uint32_t avoid_plane_id)
{
    drmModePlaneRes* plane_res = drmModeGetPlaneResources(drm_fd);
    if (!plane_res) {
        perror("drmModeGetPlaneResources");
        return 0;
    }

    int crtc_index = -1;
    for (int i = 0; i < res->count_crtcs; ++i) {
        if (res->crtcs[i] == crtc_id) {
            crtc_index = i;
            break;
        }
    }

    if (crtc_index < 0) {
        drmModeFreePlaneResources(plane_res);
        return 0;
    }

    for (uint32_t i = 0; i < plane_res->count_planes; ++i) {
        drmModePlane* plane = drmModeGetPlane(drm_fd, plane_res->planes[i]);
        if (!plane)
            continue;

        if (plane->plane_id == avoid_plane_id) {
            drmModeFreePlane(plane);
            continue;
        }

        bool crtc_ok = (plane->possible_crtcs & (1u << crtc_index)) != 0;
        bool fmt_ok = false;

        for (uint32_t j = 0; j < plane->count_formats; ++j) {
            if (plane->formats[j] == fourcc) {
                fmt_ok = true;
                break;
            }
        }

        uint32_t found = 0;
        if (crtc_ok && fmt_ok)
            found = plane->plane_id;

        drmModeFreePlane(plane);

        if (found) {
            drmModeFreePlaneResources(plane_res);
            return found;
        }
    }

    drmModeFreePlaneResources(plane_res);
    return 0;
}

bool import_dmabuf_to_fb(
    int drm_fd,
    int dmabuf_fd,
    uint32_t width,
    uint32_t height,
    uint32_t fourcc,
    const uint32_t pitches[4],
    const uint32_t offsets[4],
    uint32_t& fb_id,
    uint32_t handles_out[4]
) {
    uint32_t handles[4] = {0, 0, 0, 0};

    if (drmPrimeFDToHandle(drm_fd, dmabuf_fd, &handles[0]) != 0) {
        perror("drmPrimeFDToHandle");
        return false;
    }

    // Если все плоскости в одном dmabuf fd — handle обычно один и тот же.
    handles[1] = handles[0];
    handles[2] = handles[0];
    handles[3] = handles[0];

    if (drmModeAddFB2(
            drm_fd,
            width,
            height,
            fourcc,
            handles,
            const_cast<uint32_t*>(pitches),
            const_cast<uint32_t*>(offsets),
            &fb_id,
            0) != 0) {
        perror("drmModeAddFB2");

        struct drm_gem_close gc = {};
        gc.handle = handles[0];
        drmIoctl(drm_fd, DRM_IOCTL_GEM_CLOSE, &gc);
        return false;
    }

    handles_out[0] = handles[0];
    handles_out[1] = handles[1];
    handles_out[2] = handles[2];
    handles_out[3] = handles[3];
    return true;
}

bool drm_init(DrmContext& drm) {
    drm.fd = open("/dev/dri/card0", O_RDWR | O_CLOEXEC);
    if (drm.fd < 0) {
        perror("open /dev/dri/card0");
        return false;
    }

    drm.res = drmModeGetResources(drm.fd);
    if (!drm.res) {
        perror("drmModeGetResources");
        close(drm.fd);
        drm.fd = -1;
        return false;
    }

    for (int i = 0; i < drm.res->count_connectors; ++i) {
        drmModeConnector* c = drmModeGetConnector(drm.fd, drm.res->connectors[i]);
        if (!c)
            continue;

        if (c->connection == DRM_MODE_CONNECTED && c->count_modes > 0) {
            drm.conn = c;
            drm.conn_id = c->connector_id;
            drm.mode = c->modes[0];
            break;
        }

        drmModeFreeConnector(c);
    }

    if (!drm.conn_id || !drm.conn) {
        std::cerr << "No connected display found\n";
        return false;
    }

    drmModeEncoder* enc = nullptr;

    if (drm.conn->encoder_id)
        enc = drmModeGetEncoder(drm.fd, drm.conn->encoder_id);

    if (!enc) {
        // fallback: ищем любой encoder/CRTC
        for (int i = 0; i < drm.conn->count_encoders; ++i) {
            enc = drmModeGetEncoder(drm.fd, drm.conn->encoders[i]);
            if (enc)
                break;
        }
    }

    if (!enc) {
        std::cerr << "Failed to get encoder\n";
        return false;
    }

    drm.crtc_id = enc->crtc_id;
    drmModeFreeEncoder(enc);

    if (!drm.crtc_id) {
        std::cerr << "Failed to get CRTC\n";
        return false;
    }

    drm.old_crtc = drmModeGetCrtc(drm.fd, drm.crtc_id);
    if (!drm.old_crtc) {
        perror("drmModeGetCrtc");
        return false;
    }

    // Один раз выставим режим на экран
    if (drmModeSetCrtc(drm.fd, drm.crtc_id, 0, 0, 0, &drm.conn_id, 1, &drm.mode) != 0) {
        perror("drmModeSetCrtc");
        return false;
    }

    return true;
}

bool drm_show_dmabuf(
    DrmContext& drm,
    int dmabuf_fd,
    uint32_t width,
    uint32_t height,
    uint32_t fourcc,
    const uint32_t pitches[4],
    const uint32_t offsets[4],
    uint32_t plane_id
) {
    if (drm.fd < 0 || !drm.res || !drm.conn || !drm.crtc_id) {
        std::cerr << "DRM is not initialized\n";
        return false;
    }

    

    uint32_t fb_id = 0;
    uint32_t handles[4] = {0, 0, 0, 0};

    if (!import_dmabuf_to_fb(
            drm.fd,
            dmabuf_fd,
            width,
            height,
            fourcc,
            pitches,
            offsets,
            fb_id,
            handles)) {
        return false;
    }

    int dst_x = 0;
    int dst_y = 0;
    int dst_w = 1280;
    int dst_h = 720;

    int ret = drmModeSetPlane(
        drm.fd,
        plane_id,
        drm.crtc_id,
        fb_id,
        0,
        dst_x,
        dst_y,
        dst_w,
        dst_h,
        0 << 16,
        0 << 16,
        width << 16,
        height << 16
    );

    // // Показать на весь экран
    // int ret = drmModeSetPlane(
    //     drm.fd,
    //     plane_id,
    //     drm.crtc_id,
    //     fb_id,
    //     0,                      // flags
    //     0, 0,                   // crtc_x, crtc_y
    //     drm.mode.hdisplay,      // crtc_w
    //     drm.mode.vdisplay,      // crtc_h
    //     0 << 16,                // src_x
    //     0 << 16,                // src_y
    //     width << 16,            // src_w
    //     height << 16            // src_h
    // );

    if (ret != 0) {
        perror("drmModeSetPlane");
        drmModeRmFB(drm.fd, fb_id);

        if (handles[0]) {
            struct drm_gem_close gc = {};
            gc.handle = handles[0];
            drmIoctl(drm.fd, DRM_IOCTL_GEM_CLOSE, &gc);
        }
        return false;
    }

    // Важно:
    // Пока plane использует fb_id, удалять его сразу нельзя.
    // Если хочешь показывать следующий буфер, тогда:
    // 1) setPlane(new_fb)
    // 2) потом rm old fb + gem_close old handle

    return true;
}

static bool set_plane_prop_by_name(int drm_fd,
                                   uint32_t plane_id,
                                   const char* prop_name,
                                   uint64_t value)
{
    drmModeObjectProperties* props =
        drmModeObjectGetProperties(drm_fd, plane_id, DRM_MODE_OBJECT_PLANE);
    if (!props)
        return false;

    bool ok = false;

    for (uint32_t i = 0; i < props->count_props; ++i) {
        drmModePropertyRes* prop = drmModeGetProperty(drm_fd, props->props[i]);
        if (!prop)
            continue;

        if (strcmp(prop->name, prop_name) == 0) {
            if (drmModeObjectSetProperty(drm_fd,
                                         plane_id,
                                         DRM_MODE_OBJECT_PLANE,
                                         prop->prop_id,
                                         value) == 0) {
                ok = true;
            }
            drmModeFreeProperty(prop);
            break;
        }

        drmModeFreeProperty(prop);
    }

    drmModeFreeObjectProperties(props);
    return ok;
}

bool drm_create_overlay_plane(DrmContext& drm,
                              OverlayPlane& ovl,
                              uint32_t video_plane_id)
{
    ovl.plane_id = find_plane_for_format(drm.fd,
                                         drm.res,
                                         drm.crtc_id,
                                         DRM_FORMAT_ARGB8888,
                                         video_plane_id);
    if (!ovl.plane_id) {
        std::cerr << "No overlay plane with ARGB8888 found\n";
        return false;
    }

    ovl.width = 1280;
    ovl.height = 720;
    ovl.dst_x = 0;
    ovl.dst_y = 0;
    ovl.dst_w = ovl.width;
    ovl.dst_h = ovl.height;

    drm_mode_create_dumb create = {};
    create.width = ovl.width;
    create.height = ovl.height;
    create.bpp = 32;

    if (ioctl(drm.fd, DRM_IOCTL_MODE_CREATE_DUMB, &create) != 0) {
        perror("DRM_IOCTL_MODE_CREATE_DUMB");
        return false;
    }

    ovl.handle = create.handle;
    ovl.pitch = create.pitch;
    ovl.size = create.size;

    if (drmModeAddFB(drm.fd,
                     ovl.width,
                     ovl.height,
                     24,
                     32,
                     ovl.pitch,
                     ovl.handle,
                     &ovl.fb_id) != 0) {
        perror("drmModeAddFB");
        return false;
    }

    drm_mode_map_dumb map_dumb = {};
    map_dumb.handle = ovl.handle;
    if (ioctl(drm.fd, DRM_IOCTL_MODE_MAP_DUMB, &map_dumb) != 0) {
        perror("DRM_IOCTL_MODE_MAP_DUMB");
        return false;
    }

    ovl.map = (uint8_t*)mmap(nullptr,
                             ovl.size,
                             PROT_READ | PROT_WRITE,
                             MAP_SHARED,
                             drm.fd,
                             map_dumb.offset);
    if (ovl.map == MAP_FAILED) {
        perror("mmap overlay");
        ovl.map = nullptr;
        return false;
    }

    // Прозрачный фон
    std::memset(ovl.map, 0, ovl.size);

    // Попробуем поднять plane наверх и включить полную plane alpha
    set_plane_prop_by_name(drm.fd, ovl.plane_id, "zpos", 10);
    set_plane_prop_by_name(drm.fd, ovl.plane_id, "alpha", 0xffff);

    // Показываем overlay 1:1 на весь экран
    if (drmModeSetPlane(drm.fd,
                        ovl.plane_id,
                        drm.crtc_id,
                        ovl.fb_id,
                        0,
                        ovl.dst_x,
                        ovl.dst_y,
                        ovl.dst_w,
                        ovl.dst_h,
                        0 << 16,
                        0 << 16,
                        ovl.width << 16,
                        ovl.height << 16) != 0) {
        perror("drmModeSetPlane overlay");
        return false;
    }

    return true;
}

static inline void put_pixel_argb8888(OverlayPlane& ovl,
                                      int x, int y,
                                      uint32_t argb)
{
    if (x < 0 || y < 0 || x >= (int)ovl.width || y >= (int)ovl.height)
        return;

    uint32_t* row = (uint32_t*)(ovl.map + y * ovl.pitch);
    row[x] = argb;
}

static void draw_rect_argb8888(OverlayPlane& ovl,
                               int x, int y, int w, int h,
                               uint32_t color,
                               int thickness = 2)
{
    if (w <= 0 || h <= 0)
        return;

    for (int t = 0; t < thickness; ++t) {
        int x0 = x + t;
        int y0 = y + t;
        int x1 = x + w - 1 - t;
        int y1 = y + h - 1 - t;

        if (x0 > x1 || y0 > y1)
            break;

        for (int xx = x0; xx <= x1; ++xx) {
            put_pixel_argb8888(ovl, xx, y0, color);
            put_pixel_argb8888(ovl, xx, y1, color);
        }

        for (int yy = y0; yy <= y1; ++yy) {
            put_pixel_argb8888(ovl, x0, yy, color);
            put_pixel_argb8888(ovl, x1, yy, color);
        }
    }
}

void overlay_clear(OverlayPlane& ovl)
{
    if (!ovl.map)
        return;
    std::memset(ovl.map, 0, ovl.size); // A=0, полностью прозрачно
}

static uint32_t class_color_argb(size_t class_id)
{
    static const uint32_t colors[] = {
        0xFFFF0000, // красный
        0xFF00FF00, // зелёный
        0xFF0000FF, // синий
        0xFFFFFF00, // жёлтый
        0xFFFF00FF, // пурпурный
        0xFF00FFFF, // циан
        0xFFFF8800, // оранжевый
        0xFFFFFFFF  // белый
    };
    return colors[class_id % (sizeof(colors) / sizeof(colors[0]))];
}

template <typename T>
inline T clamp(T v, T lo, T hi)
{
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

void overlay_draw_bboxes(OverlayPlane& ovl,
                         const std::vector<NamedBbox>& boxes,
                         float score_threshold ,
                         int thickness)
{
    overlay_clear(ovl);

    for (const auto& nb : boxes) {
        const auto& b = nb.bbox;

        if (b.score < score_threshold)
            continue;

        float x0f = clamp(b.x_min, 0.0f, 1.0f);
        float y0f = clamp(b.y_min, 0.0f, 1.0f);
        float x1f = clamp(b.x_max, 0.0f, 1.0f);
        float y1f = clamp(b.y_max, 0.0f, 1.0f);

        int x = (int)(x0f * ovl.width);
        int y = (int)(y0f * ovl.height);
        int w = (int)((x1f - x0f) * ovl.width);
        int h = (int)((y1f - y0f) * ovl.height);

        if (w <= 0 || h <= 0)
            continue;

        uint32_t color = class_color_argb(nb.class_id);
        draw_rect_argb8888(ovl, x, y, w, h, color, thickness);
    }
}
void drm_init() {

    drm_fd = open("/dev/dri/card0", O_RDWR | O_CLOEXEC);

    log("Drm opened");
    if (drm_fd < 0) {
        perror("open");
        return;
    }

    res = drmModeGetResources(drm_fd);
    conn = nullptr;
    
    conn_id = 0;
    for (int i = 0; i < res->count_connectors; ++i) {
        conn = drmModeGetConnector(drm_fd, res->connectors[i]);
        if (conn->connection == DRM_MODE_CONNECTED && conn->count_modes > 0) {
            mode = conn->modes[0];
            conn_id = conn->connector_id;
            break;
        }
        drmModeFreeConnector(conn);
    }
    log("Drm connector");

    if (!conn_id) {
        std::cerr << "No connected display found\n";
        return;
    }

    drmModeEncoder* enc = drmModeGetEncoder(drm_fd, conn->encoder_id);
    crtc_id = enc->crtc_id;
    old_crtc = drmModeGetCrtc(drm_fd, crtc_id);
    log("Drm encoder");
    struct drm_mode_create_dumb create = {};
    create.width = mode.hdisplay;
    create.height = mode.vdisplay;
    create.bpp = 32;
    ioctl(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &create);
    handle = create.handle;
    pitch = create.pitch;
    size = create.size;

    struct drm_mode_map_dumb map_dumb = {};
    map_dumb.handle = handle;
    ioctl(drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &map_dumb);
    map = (uint8_t*)mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, drm_fd, map_dumb.offset);
    log("Drm mmap");
    
    fb.width = mode.hdisplay;
    fb.height = mode.vdisplay;
    fb.pitch = pitch;
    fb.bpp = 32;
    fb.depth = 24;
    fb.handle = handle;
    drmModeAddFB(drm_fd, fb.width, fb.height, fb.depth, fb.bpp, pitch, handle, &fb_id);

    drmModeSetCrtc(drm_fd, crtc_id, fb_id, 0, 0, &conn_id, 1, &mode);

} 

void drm_destroy() {

    if (old_crtc) {
        drmModeSetCrtc(drm_fd, old_crtc->crtc_id, old_crtc->buffer_id,
                       old_crtc->x, old_crtc->y,
                       &conn_id, 1, &old_crtc->mode);
        drmModeFreeCrtc(old_crtc);
    }

    munmap(map, size);
    drmModeRmFB(drm_fd, fb_id);

    struct drm_mode_destroy_dumb destroy = {0};
    destroy.handle = handle;
    drmIoctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);

    drmModeFreeConnector(conn);
    drmModeFreeResources(res);
    close(drm_fd);

}
