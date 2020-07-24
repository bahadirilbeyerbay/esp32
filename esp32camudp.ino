#include <WiFiUdp.h>
#include "WiFi.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "soc/soc.h"           
#include "soc/rtc_cntl_reg.h"  
#include "driver/rtc_io.h"
#include <ESPAsyncWebServer.h>
#include <StringArray.h>
#include <SPIFFS.h>
#include <FS.h>
#include <base64.h>

const char * ssid = "ProfArduino"; 
const char * pwd = "10dansanaNE+1973.";
const char * udpAdres = "192.168.1.105"; 
const int udpPort = 8080; 

  WiFiUDP udp;

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

#define FILE_PHOTO "/photo.jpg"
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


void setup() {
  Serial.begin(115200); 
  WiFi.begin(ssid, pwd);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  Serial.print("Wifi agina baglanildi... Ip adresi: ");
  Serial.println(WiFi.localIP());
  udp.begin(udpPort);
  
  if (!SPIFFS.begin(true)) {
    ESP.restart();
  }
  else {
    delay(500);
  }

  if (psramFound()) {
    config.frame_size = FRAMESIZE_CIF;
    config.jpeg_quality = 20;
    config.fb_count = 1;
  } else {
    config.frame_size = FRAMESIZE_CIF;
    config.jpeg_quality = 20;
    config.fb_count = 1;
  }
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    ESP.restart();
  }
}

void loop() {
    camera_fb_t * fb = NULL; 
    fb = esp_camera_fb_get();
    if (!fb) {
      return;
    }
      Serial.print("Gorsel kaydedildi...");
      Serial.println(base64::encode(fb->buf,fb->len));
      uint8_t buffer[50] = "sunucuya giden mesaj";
      udp.beginPacket(udpAdres, udpPort);
      udp.write(buffer,50);
      udp.endPacket();
      memset(buffer, 0, 50);
      udp.parsePacket();
      if(udp.read(buffer, 50) > 0){
      Serial.print("Sunucudan gelen mesaj: ");
      Serial.println((char *)buffer);
  }
    esp_camera_fb_return(fb);
}
