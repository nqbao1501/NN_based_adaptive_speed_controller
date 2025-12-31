#include "web_dashboard.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "esp_log.h"

/* ====== EXTERNAL VARIABLES FROM YOUR CODE ====== */
extern volatile float SETPOINT_RPM_SHARED;
extern portMUX_TYPE setpoint_mux;
extern float current_rpm;

/* ====== HTML DASHBOARD ====== */
static const char index_html[] =
"<!DOCTYPE html><html><head>"
"<meta charset='utf-8'>"
"<title>ESP32 Motor Dashboard</title>"
"<style>"
"body{font-family:Arial;text-align:center;background:#111;color:#eee;}"
"canvas{background:#000;border:1px solid #444;}"
".val{font-size:32px;margin:10px;}"
"</style>"
"</head><body>"
"<h1>ESP32 Motor Dashboard</h1>"
"<div class='val'>Setpoint: <span id='sp'>0</span> RPM</div>"
"<div class='val'>Actual: <span id='rpm'>0</span> RPM</div>"
"<canvas id='plot' width='800' height='300'></canvas>"

"<script>"
"const c=document.getElementById('plot');"
"const ctx=c.getContext('2d');"
"let sp=[],rpm=[];"
"const N=200;"

"function draw(){"
"ctx.clearRect(0,0,c.width,c.height);"
"ctx.strokeStyle='red';ctx.beginPath();"
"for(let i=0;i<sp.length;i++){"
"let x=i*(c.width/N);"
"let y=c.height-(sp[i]/800)*c.height;"
"if(i==0)ctx.moveTo(x,y);else ctx.lineTo(x,y);}"
"ctx.stroke();"

"ctx.strokeStyle='cyan';ctx.beginPath();"
"for(let i=0;i<rpm.length;i++){"
"let x=i*(c.width/N);"
"let y=c.height-(rpm[i]/800)*c.height;"
"if(i==0)ctx.moveTo(x,y);else ctx.lineTo(x,y);}"
"ctx.stroke();"
"}"

"function update(){"
"fetch('/data').then(r=>r.json()).then(d=>{"
"document.getElementById('sp').innerText=d.setpoint.toFixed(1);"
"document.getElementById('rpm').innerText=d.rpm.toFixed(1);"
"sp.push(d.setpoint);rpm.push(d.rpm);"
"if(sp.length>N){sp.shift();rpm.shift();}"
"draw();});}"
"setInterval(update,20);"
"</script>"
"</body></html>";

/* ====== HTTP HANDLERS ====== */
static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t data_handler(httpd_req_t *req)
{
    float sp, rpm;

    taskENTER_CRITICAL(&setpoint_mux);
    sp = SETPOINT_RPM_SHARED;
    taskEXIT_CRITICAL(&setpoint_mux);

    rpm = current_rpm;

    char buf[128];
    int len = snprintf(buf, sizeof(buf),
        "{\"setpoint\":%.2f,\"rpm\":%.2f}", sp, rpm);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, len);
    return ESP_OK;
}

/* ====== HTTP SERVER ====== */
static void http_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server;

    ESP_ERROR_CHECK(httpd_start(&server, &config));

    httpd_uri_t root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_handler
    };

    httpd_uri_t data = {
        .uri = "/data",
        .method = HTTP_GET,
        .handler = data_handler
    };

    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &data);
}

/* ====== WIFI AP ====== */
static void wifi_ap_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap_cfg = {
        .ap = {
            .ssid = "ESP32_RPM",
            .ssid_len = 0,
            .password = "12345678",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA2_PSK
        }
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
}

/* ====== PUBLIC INIT ====== */
void web_dashboard_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_ap_init();
    http_server_start();
}
