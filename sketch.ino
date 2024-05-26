/*
http://192.168.xxx.xxx     // Antarmuka manajemen beranda web
http://192.168.xxx.xxx:81/stream   // Dapatkan aliran video       <img src="http://192.168.xxx.xxx:81/stream">
http://192.168.xxx.xxx/capture     // Dapatkan gambar          <img src="http://192.168.xxx.xxx/capture">
http://192.168.xxx.xxx/status      // Dapatkan nilai parameter video

Format perintah kustom :  
http://APIP/control?cmd=P1;P2;P3;P4;P5;P6;P7;P8;P9
http://STAIP/control?cmd=P1;P2;P3;P4;P5;P6;P7;P8;P9

IP AP default: 192.168.4.1

Format perintah kustom http://192.168.xxx.xxx/control?cmd=P1;P2;P3;P4;P5;P6;P7;P8;P9
http://192.168.xxx.xxx/control?ip                      // Dapatkan APIP, STAIP
http://192.168.xxx.xxx/control?mac                     // Dapatkan alamat MAC
http://192.168.xxx.xxx/control?restart                 // Restart ESP32-CAM
http://192.168.xxx.xxx/control?digitalwrite=pin;value  // Output digital
http://192.168.xxx.xxx/control?analogwrite=pin;value   // Output analog
http://192.168.xxx.xxx/control?digitalread=pin         // Baca digital
http://192.168.xxx.xxx/control?analogread=pin          // Baca analog
http://192.168.xxx.xxx/control?touchread=pin           // Baca sentuhan
http://192.168.xxx.xxx/control?resetwifi=ssid;password   // Reset jaringan Wi-Fi
http://192.168.xxx.xxx/control?flash=value             // Lampu kilat internal value= 0~255
http://192.168.xxx.xxx/control?serial=barcode          // Tampilkan barcode serial

Format resmi http://192.168.xxx.xxx/control?var=***&val=***
http://192.168.xxx.xxx/control?var=framesize&val=value    // value = 10->UXGA(1600x1200), 9->SXGA(1280x1024), 8->XGA(1024x768) ,7->SVGA(800x600), 6->VGA(640x480), 5 selected=selected->CIF(400x296), 4->QVGA(320x240), 3->HQVGA(240x176), 0->QQVGA(160x120)
http://192.168.xxx.xxx/control?var=quality&val=value      // value = 10 ~ 63
http://192.168.xxx.xxx/control?var=brightness&val=value   // value = -2 ~ 2
http://192.168.xxx.xxx/control?var=contrast&val=value     // value = -2 ~ 2
http://192.168.xxx.xxx/control?var=hmirror&val=value      // value = 0 or 1 
http://192.168.xxx.xxx/control?var=vflip&val=value        // value = 0 or 1 
http://192.168.xxx.xxx/control?var=flash&val=value        // value = 0 ~ 255 
      
Query IP klien:
Query IP: http://192.168.4.1/?ip
*/

// Impor pustaka
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "esp_camera.h"          
#include "soc/soc.h"             
#include "soc/rtc_cntl_reg.h"   
#include "esp_http_server.h"
#include "esp_camera.h"
#include "img_converters.h"

// Menentukan SSID dan password untuk koneksi WIFI
const char* ssid = "android";
const char* password = "12345678";

// Menentukan SSID dan password untuk koneksi AP (Access Point)
const char* apssid = "esp32cam";
const char* appassword = "12345678";   // Password AP harus setidaknya 8 karakter

// Mendefinisikan variabel untuk menyimpan umpan balik
String Feedback = "";

// Mendefinisikan variabel untuk menyimpan parameter perintah yang diterima
String Command = "";
String cmd = "";
String P1 = "";
String P2 = "";
String P3 = "";
String P4 = "";
String P5 = "";
String P6 = "";
String P7 = "";
String P8 = "";
String P9 = "";

// Mendefinisikan variabel status untuk pemrosesan perintah
byte ReceiveState = 0;
byte cmdState = 1;
byte strState = 1;
byte questionstate = 0;
byte equalstate = 0;
byte semicolonstate = 0;

// Struktur untuk mengirimkan chunk JPG
typedef struct {
        httpd_req_t *req;
        size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

// Menentukan pin yang digunakan oleh modul ESP32-CAM
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 21
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 19
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 5
#define Y2_GPIO_NUM 4
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// Fungsi untuk mengubah sudut transfer
int transferAngle(int angle, String side) {
    if (angle > 180)
        angle = 180;
    else if (angle < 0)
        angle = 0;
    if (side == "right")
        angle = 180 - angle;
    return angle * 6300 / 180 + 1700;
}

// Fungsi untuk mengirimkan chunk JPG
static size_t jpg_encode_stream(void *arg, size_t index, const void* data, size_t len) {
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if (!index) {
        j->len = 0;
    }
    if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK) {
        return 0;
    }
    j->len += len;
    return len;
}

// Fungsi untuk menangani pengambilan gambar
static esp_err_t capture_handler(httpd_req_t *req) {
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;

    fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    size_t fb_len = 0;
    if (fb->format == PIXFORMAT_JPEG) {
        fb_len = fb->len;
        res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    } else {
        jpg_chunking_t jchunk = {req, 0};
        res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
        httpd_resp_send_chunk(req, NULL, 0);
        fb_len = jchunk.len;
    }
    esp_camera_fb_return(fb);
    return res;
}

// Fungsi untuk menangani streaming video
static esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    char *part_buf[64];

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera capture failed");
            res = ESP_FAIL;
        } else {
            if (fb->format != PIXFORMAT_JPEG) {
                bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                esp_camera_fb_return(fb);
                fb = NULL;
                if (!jpeg_converted) {
                    Serial.println("JPEG compression failed");
                    res = ESP_FAIL;
                }
            } else {
                _jpg_buf_len = fb->len;
                _jpg_buf = fb->buf;
            }
        }

        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if (res == ESP_OK) {
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if (fb) {
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        } else if (_jpg_buf) {
            free(_jpg_buf);
            _jpg_buf = NULL;
        }
        if (res != ESP_OK) {
            break;
        }
    }
    return res;
}

// Fungsi untuk menangani perintah dan parameter
static esp_err_t cmd_handler(httpd_req_t *req){
    char* buf; // Buffer untuk menyimpan string parameter dari URL
    size_t buf_len;
    char variable[128] = {0,}; // Buffer untuk menyimpan nilai parameter 'var'
    char value[128] = {0,}; // Buffer untuk menyimpan nilai parameter 'val'
    String myCmd = "";

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char*)malloc(buf_len);
        if(!buf){
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK &&
                httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK) {
                // Mengambil nilai parameter var dan val
            } else {
                myCmd = String(buf); // Jika tidak dalam format resmi, anggap sebagai perintah khusus
            }
        }
        free(buf);
    } else {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // Reset semua variabel
    Feedback = ""; Command = ""; cmd = ""; P1 = ""; P2 = ""; P3 = ""; P4 = ""; P5 = ""; P6 = ""; P7 = ""; P8 = ""; P9 = "";
    ReceiveState = 0; cmdState = 1; strState = 1; questionstate = 0; equalstate = 0; semicolonstate = 0;
    
    if (myCmd.length() > 0) {
        myCmd = "?" + myCmd; // Konversi string parameter URL ke format perintah khusus
        for (int i = 0; i < myCmd.length(); i++) {
            getCommand(char(myCmd.charAt(i))); // Pisahkan string parameter perintah khusus
        }
    }

    if (cmd.length() > 0) {
        Serial.println("");
        Serial.println("cmd= " + cmd + " ,P1= " + P1 + " ,P2= " + P2 + " ,P3= " + P3 + " ,P4= " + P4 + " ,P5= " + P5 + " ,P6= " + P6 + " ,P7= " + P7 + " ,P8= " + P8 + " ,P9= " + P9);
        Serial.println(""); 

        // Blok perintah khusus http://192.168.xxx.xxx/control?cmd=P1;P2;P3;P4;P5;P6;P7;P8;P9
        if (cmd == "your cmd") {
            // Anda bisa melakukan apapun di sini
            // Feedback="<font color=\"red\">Hello World</font>"; // Bisa berupa teks biasa atau HTML
        } else if (cmd == "ip") { // Perintah untuk mengecek IP AP dan STA
            Feedback = "AP IP: " + WiFi.softAPIP().toString();
            Feedback += "<br>";
            Feedback += "STA IP: " + WiFi.localIP().toString();
        } else if (cmd == "mac") { // Perintah untuk mengecek alamat MAC
            Feedback = "STA MAC: " + WiFi.macAddress();
        } else if (cmd == "restart") { // Perintah untuk merestart ESP
            ESP.restart();
        } else if (cmd == "digitalwrite") { // Perintah untuk menulis ke pin digital
            ledcDetachPin(P1.toInt());
            pinMode(P1.toInt(), OUTPUT);
            digitalWrite(P1.toInt(), P2.toInt());
        } else if (cmd == "digitalread") { // Perintah untuk membaca dari pin digital
            Feedback = String(digitalRead(P1.toInt()));
        } else if (cmd == "analogwrite") { // Perintah untuk menulis ke pin analog
            if (P1 == "4") {
                ledcAttachPin(2, 4);
                ledcSetup(4, 5000, 8);
                ledcWrite(4, P2.toInt());
            } else {
                ledcAttachPin(P1.toInt(), 9);
                ledcSetup(9, 5000, 8);
                ledcWrite(9, P2.toInt());
            }
        } else if (cmd == "analogread") { // Perintah untuk membaca dari pin analog
            Feedback = String(analogRead(P1.toInt()));
        } else if (cmd == "touchread") { // Perintah untuk membaca dari pin touch
            Feedback = String(touchRead(P1.toInt()));
        } else if (cmd == "resetwifi") { // Perintah untuk mereset koneksi WiFi
            for (int i = 0; i < 2; i++) {
                WiFi.begin(P1.c_str(), P2.c_str());
                Serial.print("Connecting to ");
                Serial.println(P1);
                long int StartTime = millis();
                while (WiFi.status() != WL_CONNECTED) {
                    delay(500);
                    if ((StartTime + 5000) < millis()) break;
                }
                Serial.println("");
                Serial.println("STAIP: " + WiFi.localIP().toString());
                Feedback = "STAIP: " + WiFi.localIP().toString();
  
                if (WiFi.status() == WL_CONNECTED) {
                    WiFi.softAP((WiFi.localIP().toString() + "_" + P1).c_str(), P2.c_str());
                    for (int i = 0; i < 2; i++) { // Jika tidak bisa terhubung, setel lampu kilat untuk berkedip lambat
                        ledcWrite(4, 10);
                        delay(300);
                        ledcWrite(4, 0);
                        delay(300);
                    }
                    break;
                }
            }
        } else if (cmd == "flash") { // Perintah untuk mengontrol lampu kilat bawaan
            ledcAttachPin(2, 4);
            ledcSetup(4, 5000, 8);
            int val = P1.toInt();
            ledcWrite(4, val);
        } else if (cmd == "serial") {
            Serial.println(P1);
        } else {
            Feedback = "Command is not defined";
        }

        if (Feedback == "") Feedback = Command; // Jika tidak ada data umpan balik, kembalikan nilai Command
    
        const char *resp = Feedback.c_str();
        httpd_resp_set_type(req, "text/html"); // Menetapkan tipe respons
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*"); // Mengizinkan akses lintas domain
        return httpd_resp_send(req, resp, strlen(resp));
    } else {
        // Blok perintah resmi, juga bisa mendefinisikan perintah di sini http://192.168.xxx.xxx/control?var=xxx&val=xxx
        int val = atoi(value);
        sensor_t *s = esp_camera_sensor_get();
        int res = 0;

        if (!strcmp(variable, "framesize")) {
            if (s->pixformat == PIXFORMAT_JPEG) 
                res = s->set_framesize(s, (framesize_t)val);
        } else if (!strcmp(variable, "quality")) res = s->set_quality(s, val);
        else if (!strcmp(variable, "contrast")) res = s->set_contrast(s, val);
        else if (!strcmp(variable, "brightness")) res = s->set_brightness(s, val);
        else if (!strcmp(variable, "hmirror")) res = s->set_hmirror(s, val);
        else if (!strcmp(variable, "vflip")) res = s->set_vflip(s, val);
        else if (!strcmp(variable, "flash")) { // Perintah lampu kilat khusus
            ledcAttachPin(2, 4);
            ledcSetup(4, 5000, 8);
            ledcWrite(4, val);
        } else {
            res = -1;
        }
  
        if (res) {
            return httpd_resp_send_500(req);
        }

        if (buf) {
            Feedback = String(buf);
            const char *resp = Feedback.c_str();
            httpd_resp_set_type(req, "text/html");
            httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
            return httpd_resp_send(req, resp, strlen(resp)); // Mengirimkan string parameter sebagai respons
        } else {
            httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
            return httpd_resp_send(req, NULL, 0);
        }
    }
}

// Fungsi untuk menghasilkan nada pada pin tertentu dengan frekuensi dan durasi tertentu
void tone(int pin, int frequency, int duration) {
    ledcSetup(9, 2000, 8); // Mengatur saluran LED PWM dengan frekuensi 2000 Hz dan resolusi 8 bit
    ledcAttachPin(pin, 9); // Menghubungkan pin ke saluran LED PWM
    ledcWriteTone(9, frequency); // Menulis frekuensi nada ke saluran LED PWM
    delay(duration); // Menunggu selama durasi yang ditentukan
    ledcWriteTone(9, 0); // Menghentikan nada dengan menulis frekuensi 0
}

// Menampilkan status parameter video (harus mengembalikan format json untuk memuat pengaturan awal)
static esp_err_t status_handler(httpd_req_t *req){
    static char json_response[1024]; // Buffer untuk menyimpan respons JSON

    sensor_t * s = esp_camera_sensor_get(); // Mendapatkan pointer ke sensor kamera
    char * p = json_response;
    *p++ = '{';
    p += sprintf(p, "\"flash\":%d,", 0); // Menambahkan status flash ke JSON
    p += sprintf(p, "\"framesize\":%u,", s->status.framesize); // Menambahkan ukuran frame ke JSON
    p += sprintf(p, "\"quality\":%u,", s->status.quality); // Menambahkan kualitas ke JSON
    p += sprintf(p, "\"brightness\":%d,", s->status.brightness); // Menambahkan kecerahan ke JSON
    p += sprintf(p, "\"contrast\":%d,", s->status.contrast); // Menambahkan kontras ke JSON
    p += sprintf(p, "\"hmirror\":%u", s->status.hmirror); // Menambahkan status horizontal mirror ke JSON
    *p++ = '}';
    *p++ = 0;
    httpd_resp_set_type(req, "application/json"); // Menetapkan tipe respons sebagai JSON
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*"); // Mengizinkan akses lintas domain
    return httpd_resp_send(req, json_response, strlen(json_response)); // Mengirimkan respons JSON
}

//自訂網頁首頁
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!doctype html>
<html>
    <head>
        <meta charset="utf-8">
        <meta name="viewport" content="width=device-width,initial-scale=1">
        <title>ESP32 OV2460</title>
        <style>        
            body {
                font-family: Arial,Helvetica,sans-serif;
                background: #F0F0F0;
                color: #000;
                font-size: 16px
            }
            h2 {
                font-size: 18px
            }
            section.main {
                display: flex
            }
            #menu,section.main {
                flex-direction: column
            }
            #menu {
                display: block;
                flex-wrap: nowrap;
                min-width: 340px;
                background: #fff;
                padding: 8px;
                border-radius: 4px;
                margin-top: -10px;
                margin-right: 10px;
            }
            #content {
                display: flex;
                flex-wrap: wrap;
                align-items: stretch
            }
            figure {
                padding: 0px;
                margin: 0;
                -webkit-margin-before: 0;
                margin-block-start: 0;
                -webkit-margin-after: 0;
                margin-block-end: 0;
                -webkit-margin-start: 0;
                margin-inline-start: 0;
                -webkit-margin-end: 0;
                margin-inline-end: 0
            }
            figure img {
                display: block;
                width: 100%;
                height: auto;
                border-radius: 4px;
                margin-top: 8px;
            }
            @media (min-width: 800px) and (orientation:landscape) {
                #content {
                    display:flex;
                    flex-wrap: nowrap;
                    align-items: stretch
                }
                figure img {
                    display: block;
                    max-width: 100%;
                    max-height: calc(100vh - 40px);
                    width: auto;
                    height: auto
                }
                figure {
                    padding: 0 0 0 0px;
                    margin: 0;
                    -webkit-margin-before: 0;
                    margin-block-start: 0;
                    -webkit-margin-after: 0;
                    margin-block-end: 0;
                    -webkit-margin-start: 0;
                    margin-inline-start: 0;
                    -webkit-margin-end: 0;
                    margin-inline-end: 0
                }
            }
            section#buttons {
                display: flex;
                flex-wrap: nowrap;
                justify-content: space-between
            }
            #nav-toggle {
                cursor: pointer;
                display: block
            }
            #nav-toggle-cb {
                outline: 0;
                opacity: 0;
                width: 0;
                height: 0
            }
            #nav-toggle-cb:checked+#menu {
                display: none
            }
            .input-group {
                display: flex;
                flex-wrap: nowrap;
                line-height: 22px;
                margin: 5px 0
            }
            .input-group>label {
                display: inline-block;
                padding-right: 10px;
                min-width: 47%
            }
            .input-group input,.input-group select {
                flex-grow: 1
            }
            .range-max,.range-min {
                display: inline-block;
                padding: 0 5px
            }
            button {
                display: block;
                margin: 5px;
                padding: 0 12px;
                border: 0;
                line-height: 28px;
                cursor: pointer;
                color: #fff;
                background: blue;
                border-radius: 5px;
                font-size: 16px;
                outline: 0
            }
            button:hover {
                background: blue
            }
            button:active {
                background: blue
            }
            button.disabled {
                cursor: default;
                background: #a0a0a0
            }
            input[type=range] {
                -webkit-appearance: none;
                width: 100%;
                height: 22px;
                background: #F0F0F0;
                cursor: pointer;
                margin: 0
            }
            input[type=range]:focus {
                outline: 0
            }
            input[type=range]::-webkit-slider-runnable-track {
                width: 100%;
                height: 2px;
                cursor: pointer;
                background: black;
                border-radius: 0;
                border: 0 solid #F0F0F0
            }
            input[type=range]::-webkit-slider-thumb {
                border: 1px solid rgba(0,0,30,0);
                height: 22px;
                width: 22px;
                border-radius: 50px;
                background: blue;
                cursor: pointer;
                -webkit-appearance: none;
                margin-top: -11.5px
            }
            input[type=range]:focus::-webkit-slider-runnable-track {
                background: black
            }
            input[type=range]::-moz-range-track {
                width: 100%;
                height: 2px;
                cursor: pointer;
                background: black;
                border-radius: 0;
                border: 0 solid black
            }
            input[type=range]::-moz-range-thumb {
                border: 1px solid rgba(0,0,30,0);
                height: 22px;
                width: 22px;
                border-radius: 50px;
                background: blue;
                cursor: pointer
            }
            input[type=range]::-ms-track {
                width: 100%;
                height: 2px;
                cursor: pointer;
                background: 0 0;
                border-color: transparent;
                color: transparent
            }
            input[type=range]::-ms-fill-lower {
                background: black;
                border: 0 solid black;
                border-radius: 0
            }
            input[type=range]::-ms-fill-upper {
                background: blsck;
                border: 0 solid black;
                border-radius: 0
            }
            input[type=range]::-ms-thumb {
                border: 1px solid rgba(0,0,30,0);
                height: 22px;
                width: 22px;
                border-radius: 50px;
                background: blue;
                cursor: pointer;
                height: 2px
            }
            input[type=range]:focus::-ms-fill-lower {
                background: #F0F0F0
            }
            input[type=range]:focus::-ms-fill-upper {
                background: grey
            }
            .switch {
                display: block;
                position: relative;
                line-height: 22px;
                font-size: 16px;
                height: 22px
            }
            .switch input {
                outline: 0;
                opacity: 0;
                width: 0;
                height: 0
            }
            .slider {
                width: 50px;
                height: 22px;
                border-radius: 22px;
                cursor: pointer;
                background-color: grey
            }
            .slider,.slider:before {
                display: inline-block;
                transition: .4s
            }
            .slider:before {
                position: relative;
                content: "";
                border-radius: 50%;
                height: 16px;
                width: 16px;
                left: 4px;
                top: 3px;
                background-color: white
            }
            input:checked+.slider {
                background-color: blue
            }
            input:checked+.slider:before {
                -webkit-transform: translateX(26px);
                transform: translateX(26px)
            }
            select {
                border: 1px solid #F0F0F0
                font-size: 14px;
                height: 22px;
                outline: 0;
                border-radius: 4px
            }
            .image-container {
                position: relative;
                min-width: 160px
            }
            .close {
                position: absolute;
                right: 5px;
                top: 5px;
                background: blue;
                width: 16px;
                height: 16px;
                border-radius: 100px;
                color: white;
                text-align: center;
                line-height: 18px;
                cursor: pointer
            }
            .hidden {
                display: none
            }
        </style>
        <script src="https:\/\/aos92.github.io/ESP32CAM-BarCode-Reader/barcode-reader.min.js"></script>     
    </head>
    <body>
    <figure>
      <div id="stream-container" class="image-container hidden">
        <div class="close" id="close-stream">×</div>
        <img id="stream" src="" crossorigin="anonymous">
        Code:
        <select id="code">
          <option value="ean-13">EAN-13</option>
          <option value="ean-8">EAN-8</option>
          <option value="code-39" selected>Code-39</option>
          <option value="code-2of5">Code-2of5</option>
          <option value="codabar">Codabar</option>
          <option value="code-128">Code-128</option>
        </select>
      </div>
    </figure>
        <section class="main">
            <section id="buttons">
                <table>
                <tr><td><button id="restart" onclick="try{fetch(document.location.origin+'/control?restart');}catch(e){}">Restart</button></td><td><button id="toggle-stream">Start Stream</button></td><td><button id="face_enroll" style="display:none" class="disabled" disabled="disabled"></button><button id="get-still" style="display:none">get-still</button></td></tr>
                <tr><td>Flash</td><td colspan="2"><input type="range" id="flash" min="0" max="255" value="0" onchange="try{fetch(document.location.origin+'/control?flash='+this.value);}catch(e){}"></td></tr>
                <tr style="display:none"><td colspan="3"></td></tr> 
                </table>
            </section>         
            <div id="logo">
                <label for="nav-toggle-cb" id="nav-toggle">&#9776;&nbsp;&nbsp;Toggle settings</label>
            </div>
            <div id="content">
                <div id="sidebar">
                    <input type="checkbox" id="nav-toggle-cb">
                    <nav id="menu">
                        <div class="input-group" id="flash-group">
                            <label for="flash">Flash</label>
                            <div class="range-min">0</div>
                            <input type="range" id="flash" min="0" max="255" value="0" class="default-action">
                            <div class="range-max">255</div>
                        </div>
                        <div class="input-group" id="framesize-group">
                            <label for="framesize">Resolution</label>
                            <select id="framesize" class="default-action">
                               <option value="10">UXGA(1600x1200)</option>
                                <option value="9">SXGA(1280x1024)</option>
                                <option value="8">XGA(1024x768)</option>
                                <option value="7">SVGA(800x600)</option>
                                <option value="6">VGA(640x480)</option>
                                <option value="5" selected>CIF(400x296)</option>
                                <option value="4">QVGA(320x240)</option>
                                <option value="3">HQVGA(240x176)</option>
                                <option value="0">QQVGA(160x120)</option>
                            </select>
                        </div>
                        <div class="input-group" id="quality-group">
                            <label for="quality">Quality</label>
                            <div class="range-min">10</div>
                            <input type="range" id="quality" min="10" max="63" value="10" class="default-action">
                            <div class="range-max">63</div>
                        </div>
                        <div class="input-group" id="brightness-group">
                            <label for="brightness">Brightness</label>
                            <div class="range-min">-2</div>
                            <input type="range" id="brightness" min="-2" max="2" value="0" class="default-action">
                            <div class="range-max">2</div>
                        </div>
                        <div class="input-group" id="contrast-group">
                            <label for="contrast">Contrast</label>
                            <div class="range-min">-2</div>
                            <input type="range" id="contrast" min="-2" max="2" value="0" class="default-action">
                            <div class="range-max">2</div>
                        </div>
                        <div class="input-group" id="hmirror-group">
                            <label for="hmirror">H-Mirror</label>
                            <div class="switch">
                                <input id="hmirror" type="checkbox" class="default-action" checked="checked">
                                <label class="slider" for="hmirror"></label>
                            </div>
                        </div>
                    </nav>
                </div>
            </div>
        </section>
        <div id="result" style="color:red; font-size:25px; font-weight:600;">Please wait for loading model..</div>    
        <script>
          document.addEventListener('DOMContentLoaded', function (event) {
            var baseHost = document.location.origin
            var streamUrl = baseHost + ':81'
            const hide = el => {
              el.classList.add('hidden')
            }
            const show = el => {
              el.classList.remove('hidden')
            }
            const disable = el => {
              el.classList.add('disabled')
              el.disabled = true
            }
            const enable = el => {
              el.classList.remove('disabled')
              el.disabled = false
            }
            const updateValue = (el, value, updateRemote) => {
              updateRemote = updateRemote == null ? true : updateRemote
              let initialValue
              if (el.type === 'checkbox') {
                initialValue = el.checked
                value = !!value
                el.checked = value
              } else {
                initialValue = el.value
                el.value = value
              }
              if (updateRemote && initialValue !== value) {
                updateConfig(el);
              } 
            }
            function updateConfig (el) {
              let value
              switch (el.type) {
                case 'checkbox':
                  value = el.checked ? 1 : 0
                  break
                case 'range':
                case 'select-one':
                  value = el.value
                  break
                case 'button':
                case 'submit':
                  value = '1'
                  break
                default:
                  return
              }
              const query = `${baseHost}/control?var=${el.id}&val=${value}`
              fetch(query)
                .then(response => {
                  console.log(`request to ${query} finished, status: ${response.status}`)
                })
            }
            document
              .querySelectorAll('.close')
              .forEach(el => {
                el.onclick = () => {
                  hide(el.parentNode)
                }
              })
            // read initial values
            fetch(`${baseHost}/status`)
              .then(function (response) {
                return response.json()
              })
              .then(function (state) {
                document
                  .querySelectorAll('.default-action')
                  .forEach(el => {
                    updateValue(el, state[el.id], false)
                  })
              })
            const view = document.getElementById('stream')
            const viewContainer = document.getElementById('stream-container')
            const stillButton = document.getElementById('get-still')
            const streamButton = document.getElementById('toggle-stream')
            const closeButton = document.getElementById('close-stream')
            const stopStream = () => {
              window.stop();
              streamButton.innerHTML = 'Start Stream'
            }
            const startStream = () => {
              view.src = `${streamUrl}/stream`
              show(viewContainer)
              streamButton.innerHTML = 'Stop Stream'
            }
            // Attach actions to buttons
            stillButton.onclick = () => {
              stopStream()
              try{
                view.src = `${baseHost}/capture?_cb=${Date.now()}`
              }
              catch(e) {
                view.src = `${baseHost}/capture?_cb=${Date.now()}`  
              }
              show(viewContainer)
            }
            closeButton.onclick = () => {
              stopStream()
              hide(viewContainer)
            }
            streamButton.onclick = () => {
              const streamEnabled = streamButton.innerHTML === 'Stop Stream'
              if (streamEnabled) {
                stopStream()
              } else {
                startStream()
              }
            }
            // Attach default on change action
            document
              .querySelectorAll('.default-action')
              .forEach(el => {
                el.onchange = () => updateConfig(el)
              })
          })
        </script>
        <script>
          var stream = document.getElementById('stream');
          var result = document.getElementById('result');
          var code = document.getElementById('code');
           
          result.innerHTML = "";
      
          setInterval(function(){
            stream.style = "border-width:3px;border-style:solid;border-color:red;";
            setTimeout(function(){ stream.style = ""; }, 1000);
            result.innerHTML = "";
            
            javascriptBarcodeReader({
              /* Image file Path || {data: Uint8ClampedArray, width, height} || HTML5 Canvas ImageData */
              image: stream,
              barcode: code.value,  // EAN-13, EAN-8, Code-39, Code-93, Code-2of5, Codabar, Code-128 (UCC/EAN-128)
              // barcodeType: 'industrial',
              options: {
              // useAdaptiveThreshold: true
              // singlePass: true
              }
            })
            .then(code => {
              console.log(code);
              result.innerHTML = code;
              
              fetch(document.location.origin+'/control?serial='+code); 
            })
            .catch(err => {
              console.log(err)
              result.innerHTML = "";
            })
          }, 5000);      
        </script>
    </body>
</html>        
)rawliteral";

// Handler untuk halaman utama   http://192.168.xxx.xxx
static esp_err_t index_handler(httpd_req_t *req){
    httpd_resp_set_type(req, "text/html"); // Menetapkan tipe respons sebagai HTML
    return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML)); // Mengirimkan halaman HTML
}

// Fungsi untuk memulai server kamera dengan URL yang ditentukan
void startCameraServer(){
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();  // Mengatur konfigurasi default untuk server HTTP

    // Menentukan URL root (/) untuk halaman utama
    httpd_uri_t index_uri = {
        .uri       = "/",               // URL path
        .method    = HTTP_GET,          // Metode HTTP GET
        .handler   = index_handler,     // Handler untuk menangani request
        .user_ctx  = NULL               // Konteks pengguna (tidak digunakan)
    };

    // Menentukan URL /status untuk status kamera
    httpd_uri_t status_uri = {
        .uri       = "/status",         // URL path
        .method    = HTTP_GET,          // Metode HTTP GET
        .handler   = status_handler,    // Handler untuk menangani request
        .user_ctx  = NULL               // Konteks pengguna (tidak digunakan)
    };

    // Menentukan URL /control untuk kontrol kamera
    httpd_uri_t cmd_uri = {
        .uri       = "/control",        // URL path
        .method    = HTTP_GET,          // Metode HTTP GET
        .handler   = cmd_handler,       // Handler untuk menangani request
        .user_ctx  = NULL               // Konteks pengguna (tidak digunakan)
    }; 

    // Menentukan URL /capture untuk menangkap gambar
    httpd_uri_t capture_uri = {
        .uri       = "/capture",        // URL path
        .method    = HTTP_GET,          // Metode HTTP GET
        .handler   = capture_handler,   // Handler untuk menangani request
        .user_ctx  = NULL               // Konteks pengguna (tidak digunakan)
    };

    // Menentukan URL /stream untuk streaming video
    httpd_uri_t stream_uri = {
        .uri       = "/stream",         // URL path
        .method    = HTTP_GET,          // Metode HTTP GET
        .handler   = stream_handler,    // Handler untuk menangani request
        .user_ctx  = NULL               // Konteks pengguna (tidak digunakan)
    };

    Serial.printf("Starting web server on port: '%d'\n", config.server_port);  // Menampilkan pesan bahwa server web sedang dimulai
    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        // Mendaftarkan handler untuk setiap URL yang ditentukan
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &cmd_uri);
        httpd_register_uri_handler(camera_httpd, &status_uri);
        httpd_register_uri_handler(camera_httpd, &capture_uri);
    }

    config.server_port += 1;  // Menambah satu untuk port server stream
    config.ctrl_port += 1;    // Menambah satu untuk port UDP
    Serial.printf("Starting stream server on port: '%d'\n", config.server_port);  // Menampilkan pesan bahwa server stream sedang dimulai
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);  // Mendaftarkan handler untuk URL stream
    }
}

// Fungsi untuk memecah string perintah kustom dan memasukkannya ke variabel
void getCommand(char c) {
    if (c == '?') ReceiveState = 1; // Memulai penerimaan perintah saat ditemukan tanda '?'
    if ((c == ' ') || (c == '\r') || (c == '\n')) ReceiveState = 0; // Menghentikan penerimaan perintah pada karakter tertentu

    if (ReceiveState == 1) {
        Command = Command + String(c); // Menambahkan karakter ke string Command

        if (c == '=') cmdState = 0; // Mengatur cmdState menjadi 0 saat ditemukan tanda '='
        if (c == ';') strState++; // Meningkatkan strState saat ditemukan tanda ';'

        // Memecah string berdasarkan karakter dan status
        if ((cmdState == 1) && ((c != '?') || (questionstate == 1))) cmd = cmd + String(c);
        if ((cmdState == 0) && (strState == 1) && ((c != '=') || (equalstate == 1))) P1 = P1 + String(c);
        if ((cmdState == 0) && (strState == 2) && (c != ';')) P2 = P2 + String(c);
        if ((cmdState == 0) && (strState == 3) && (c != ';')) P3 = P3 + String(c);
        if ((cmdState == 0) && (strState == 4) && (c != ';')) P4 = P4 + String(c);
        if ((cmdState == 0) && (strState == 5) && (c != ';')) P5 = P5 + String(c);
        if ((cmdState == 0) && (strState == 6) && (c != ';')) P6 = P6 + String(c);
        if ((cmdState == 0) && (strState == 7) && (c != ';')) P7 = P7 + String(c);
        if ((cmdState == 0) && (strState == 8) && (c != ';')) P8 = P8 + String(c);
        if ((cmdState == 0) && (strState >= 9) && ((c != ';') || (semicolonstate == 1))) P9 = P9 + String(c);

        if (c == '?') questionstate = 1; // Menandakan bahwa '?' telah ditemukan
        if (c == '=') equalstate = 1; // Menandakan bahwa '=' telah ditemukan
        if ((strState >= 9) && (c == ';')) semicolonstate = 1; // Menandakan bahwa ';' telah ditemukan pada posisi strState >= 9
    }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // Mematikan pengaturan restart otomatis ketika ada masalah daya

  Serial.begin(115200);
  Serial.setDebugOutput(true);  // Mengaktifkan output debug
  Serial.println();

  // Konfigurasi kamera https://github.com/espressif/esp32-camera/blob/master/driver/include/esp_camera.h
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
  config.xclk_freq_hz = 20000000;  // Frekuensi XCLK
  config.pixel_format = PIXFORMAT_JPEG;  // Format piksel JPEG

  // Peringatan! IC PSRAM diperlukan untuk resolusi UXGA dan kualitas JPEG tinggi
  // Pastikan modul ESP32 Wrover atau papan lain dengan PSRAM dipilih
  // Gambar parsial akan dikirim jika gambar melebihi ukuran buffer

  // Jika IC PSRAM ada, inisialisasi dengan resolusi UXGA dan kualitas JPEG yang lebih tinggi
  // untuk buffer frame yang lebih besar yang telah dialokasikan sebelumnya.
  if(psramFound()){  // Memeriksa apakah ada memori PSRAM (Psuedo SRAM)
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Inisialisasi kamera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Inisialisasi kamera gagal dengan kesalahan 0x%x", err);
    ESP.restart();
  }

  // Mengatur ukuran frame awal kamera
  sensor_t * s = esp_camera_sensor_get();
  // Sensor awal dibalik secara vertikal dan warnanya agak jenuh
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);  // Membalik vertikal
    s->set_brightness(s, 1);  // Menaikkan kecerahan sedikit
    s->set_saturation(s, -2);  // Menurunkan saturasi
  }
  // Menurunkan ukuran frame untuk kecepatan frame awal yang lebih tinggi
  s->set_framesize(s, FRAMESIZE_CIF);  // Resolusi: CIF (400x296)

  // Mengatur flip vertikal
  // s->set_vflip(s, 1);
  // Mengatur mirror horizontal
  // s->set_hmirror(s, 1);

  // Mengatur pin untuk lampu flash (GPIO2)
  ledcAttachPin(2, 4);  
  ledcSetup(4, 5000, 8);
  
  WiFi.mode(WIFI_AP_STA);  // Mengatur mode WiFi (opsi lain: WIFI_AP, WIFI_STA)

  // Mengatur alamat IP statis untuk client
  // WiFi.config(IPAddress(192, 168, 201, 100), IPAddress(192, 168, 201, 2), IPAddress(255, 255, 255, 0));

  for (int i = 0; i < 2; i++) {
    WiFi.begin(ssid, password);  // Memulai koneksi WiFi
    
    delay(1000);
    Serial.println("");
    Serial.print("Menghubungkan ke ");
    Serial.println(ssid);
    
    long int StartTime = millis();
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      if ((StartTime + 5000) < millis()) break;  // Menunggu 10 detik untuk koneksi
    }

    if (WiFi.status() == WL_CONNECTED) {  // Jika berhasil terhubung
      WiFi.softAP((WiFi.localIP().toString() + "_" + (String)apssid).c_str(), appassword);  // Mengatur SSID untuk menampilkan IP client         
      Serial.println("");
      Serial.println("Alamat IP STA: ");
      Serial.println(WiFi.localIP());
      Serial.println("");
  
      for (int i = 0; i < 5; i++) {  // Lampu flash berkedip cepat jika terhubung
        ledcWrite(2, 10);
        delay(200);
        ledcWrite(2, 0);
        delay(200);    
      }
      break;
    }
  } 

  if (WiFi.status() != WL_CONNECTED) {  // Jika gagal terhubung
    WiFi.softAP((WiFi.softAPIP().toString() + "_" + (String)apssid).c_str(), appassword);         

    for (int i = 0; i < 2; i++) {  // Lampu flash berkedip lambat jika tidak terhubung
      ledcWrite(2, 10);
      delay(1000);
      ledcWrite(2, 0);
      delay(1000);    
    }
  } 
  
  // Mengatur alamat IP untuk AP
  // WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  Serial.println("");
  Serial.println("Alamat IP AP: ");
  Serial.println(WiFi.softAPIP());  
  Serial.println("");
  
  startCameraServer();  // Memulai server kamera

  // Mengatur lampu flash ke tegangan rendah
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW); 
}

void loop() {

}

