#ifndef picture_h
#define picture_h
#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "Access_point.h"
// Configuration Caméra pour AI-Thinker
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

void init_cam();
void draw_line(uint8_t* buf, int w, int x0, int y0, int x1, int y1);
void draw_debug_rect(uint8_t* buf, int w, int x1, int y1, int x2, int y2);
void binarize_and_histY(camera_fb_t* fb, uint8_t* rgb_buf, uint16_t* hY) ;
void process_and_draw_aruco(camera_fb_t* fb, uint8_t* rgb_buf, uint16_t xSeed, uint16_t ySeed);
void process_y_band(camera_fb_t* fb, uint8_t* rgb_buf, uint16_t yS, uint16_t yE, uint16_t xS, uint16_t xE);
void check_for_commands();
void video_handler();
static esp_err_t stream_handler(httpd_req_t* req);
static esp_err_t index_handler(httpd_req_t* req);
static esp_err_t seuil_handler(httpd_req_t* req);
void startCameraServer();
void stopCameraServer();
bool getWifiMode();
void setWifiMode(bool state);
#endif