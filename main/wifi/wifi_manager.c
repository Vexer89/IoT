#include "wifi_manager.h"
#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "lwip/ip4_addr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_http_server.h>

// Ten plik: #include "nvs_manager.h" -> jest już w wifi_manager.h

static const char* TAG = "WIFI_MANAGER";

static nvs_wifi_config_t s_current_cfg;
static bool s_sta_connected = false; 

/* Handler obsługujący zapis konfiguracji */
static esp_err_t save_wifi_handler(httpd_req_t *req)
{
    // 1. Odczyt danych z requestu (np. z formularza x-www-form-urlencoded)
    char content[100];
    int total_len = req->content_len;
    int cur_len = 0;
    int received = 0;
    if (total_len >= sizeof(content)) {
        // Za duże dane...
        return ESP_FAIL;
    }
    while (received < total_len) {
        int ret = httpd_req_recv(req, content + received, total_len - received);
        if (ret <= 0) {
            return ESP_FAIL;
        }
        received += ret;
    }
    content[received] = '\0';

    // Zakładamy, że wysłane dane to np. "ssid=MojaSiec&pass=MojeHaslo"
    char ssid[32] = {0};
    char pass[64] = {0};

    // Prosta ekstrakcja parametrów. W praktyce lepiej użyć funkcji do parsowania URL lub form.
    // UWAGA: to bardzo uproszczony przykład!
    char *ssid_ptr = strstr(content, "ssid=");
    char *pass_ptr = strstr(content, "pass=");
    if (ssid_ptr) {
        ssid_ptr += 5; // przeskakujemy "ssid="
        char *ampersand = strchr(ssid_ptr, '&');
        if (ampersand) {
            *ampersand = '\0'; // ucinamy
        }
        strncpy(ssid, ssid_ptr, sizeof(ssid) - 1);
    }
    if (pass_ptr) {
        pass_ptr += 5; // przeskakujemy "pass="
        // do końca linii jest hasło
        strncpy(pass, pass_ptr, sizeof(pass) - 1);
    }

    ESP_LOGI("CONFIG", "Otrzymano SSID=%s, PASS=%s", ssid, pass);

    // 2. Wywołanie wifi_manager_set_config(...)
    nvs_wifi_config_t new_cfg = {0};
    strncpy(new_cfg.wifi_ssid, ssid, sizeof(new_cfg.wifi_ssid));
    strncpy(new_cfg.wifi_pass, pass, sizeof(new_cfg.wifi_pass));

    esp_err_t err = wifi_manager_set_config(&new_cfg);
    if (err == ESP_OK) {
        // Uwaga: wifi_manager_set_config() wykona esp_restart()
        // więc w praktyce dalej się już nie wykona
        return ESP_OK;
    } else {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Save Wi-Fi config failed");
        return ESP_FAIL;
    }
}

/* Handler serwujący stronę HTML z formularzem */
static esp_err_t config_page_handler(httpd_req_t *req)
{
    const char *resp_str = "<!DOCTYPE html>"
                           "<html>"
                           "<body>"
                           "<h2>Konfiguracja Wi-Fi</h2>"
                           "<form action=\"/save_wifi\" method=\"POST\">"
                           "SSID: <input type=\"text\" name=\"ssid\"><br><br>"
                           "Password: <input type=\"password\" name=\"pass\"><br><br>"
                           "<input type=\"submit\" value=\"Zapisz\">"
                           "</form>"
                           "</body>"
                           "</html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, resp_str, strlen(resp_str));

    return ESP_OK;
}

static httpd_handle_t start_config_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        // Konfigurujemy endpoint dla strony głównej
        httpd_uri_t config_page_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = config_page_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &config_page_uri);

        // Konfigurujemy endpoint dla zapisu konfiguracji
        httpd_uri_t save_wifi_uri = {
            .uri       = "/save_wifi",
            .method    = HTTP_POST,
            .handler   = save_wifi_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &save_wifi_uri);

        return server;
    }

    return NULL;
}

/************************************************************************
 *  Obsługa zdarzeń Wi-Fi
 ************************************************************************/
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGW(TAG, "Wi-Fi disconnected, retrying...");
                esp_wifi_connect();
                s_sta_connected = false;
                break;
            default:
                break;
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
            char ip_str[16]; // Buffer to hold the IP address as a string
            esp_ip4addr_ntoa(&event->ip_info.ip, ip_str, sizeof(ip_str));
            ESP_LOGI(TAG, "Got IP: %s", ip_str);
            s_sta_connected = true;
        }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "AP mode is up. Starting HTTP server...");
        start_config_server();
    }

    }
}

/************************************************************************
 *  Funkcja uruchamiająca STA (normalny tryb)
 ************************************************************************/
static esp_err_t wifi_init_sta(const nvs_wifi_config_t* cfg)
{
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    if (!sta_netif) {
        ESP_LOGE(TAG, "Failed to create default AP netif");
        return ESP_FAIL;
    }

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    // Rejestracja handlerów zdarzeń
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                              &wifi_event_handler, NULL));

    wifi_config_t wifi_sta_cfg = {0};
    strncpy((char*)wifi_sta_cfg.sta.ssid,     cfg->wifi_ssid, sizeof(wifi_sta_cfg.sta.ssid));
    strncpy((char*)wifi_sta_cfg.sta.password, cfg->wifi_pass, sizeof(wifi_sta_cfg.sta.password));

    ESP_LOGI(TAG, "Start Wi-Fi STA, SSID=%s", wifi_sta_cfg.sta.ssid);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_sta_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    return ESP_OK;
}

/************************************************************************
 *  Funkcja uruchamiająca AP (tryb konfiguracyjny)
 ************************************************************************/
static esp_err_t wifi_init_ap(void)
{
    ESP_LOGI(TAG, "Initializing Wi-Fi AP mode...");

    // Utworzenie interfejsu sieciowego AP (ważne dla stosu lwIP)
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    if (!ap_netif) {
        ESP_LOGE(TAG, "Failed to create default AP netif");
        return ESP_FAIL;
    }

    // Konfiguracja adresu IP dla AP
    esp_netif_ip_info_t ap_ip_info;
    IP4_ADDR(&ap_ip_info.ip, 192, 168, 1, 1);   // Ustawiamy IP: 192.168.1.1
    IP4_ADDR(&ap_ip_info.gw, 192, 168, 1, 1);   // Brama taka sama jak IP
    IP4_ADDR(&ap_ip_info.netmask, 255, 255, 255, 0);  // Maska podsieci

    // Zatrzymanie serwera DHCP przed zmianą IP
    // ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));
    // ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ap_ip_info));
    // ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

    ESP_LOGI(TAG, "AP IP set to 192.168.1.1");

    // Inicjalizacja Wi-Fi
    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    // Obsługa zdarzeń Wi-Fi
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              &wifi_event_handler, NULL));

    wifi_config_t ap_config = {0};
    strncpy((char*)ap_config.ap.ssid, "ESP32_Config", sizeof(ap_config.ap.ssid));
    ap_config.ap.ssid_len = strlen("ESP32_Config");
    ap_config.ap.channel = 1;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_OPEN; // Sieć otwarta, brak hasła

    ESP_LOGI(TAG, "Starting Wi-Fi AP: SSID=ESP32_Config");
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    //ESP_LOGI(TAG, "Wi-Fi AP started successfully");

    vTaskDelay(pdMS_TO_TICKS(2000)); // Odczekaj chwilę na stabilizację Wi-Fi

    start_config_server();
    return ESP_OK;
}


/************************************************************************
 *  Główne funkcje biblioteki
 ************************************************************************/
esp_err_t wifi_manager_init(bool reset_config)
{
    if (!reset_config)
    {
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    }
    
    // Upewnij się, że NVS jest gotowe
    esp_err_t err = nvs_manager_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(err));
        return err;
    }

    // Jeśli chcemy wymusić reset, czyścimy NVS i startujemy w AP
    if (reset_config) {
        ESP_LOGW(TAG, "Forcing reset_config -> clearing NVS and starting AP");
        nvs_manager_clear_wifi_config();
        memset(&s_current_cfg, 0, sizeof(s_current_cfg));
        wifi_init_ap();
        // (Opcjonalnie) start_config_server(); // prosty serwer HTTP do konfiguracji
        return ESP_OK;
    }

    // Próba odczytu z NVS
    err = nvs_manager_get_wifi_config(&s_current_cfg);
    if (err == ESP_OK && strlen(s_current_cfg.wifi_ssid) > 0) {
        // Mamy zapisane SSID – uruchamiamy STA
        wifi_init_sta(&s_current_cfg);
    } else {
        // Brak zapisanej konfiguracji – start w AP
        memset(&s_current_cfg, 0, sizeof(s_current_cfg));
        wifi_init_ap();
        // (Opcjonalnie) start_config_server();
        //start_config_server();

    }

    return ESP_OK;
}

esp_err_t wifi_manager_get_config(nvs_wifi_config_t *out_cfg)
{
    if (!out_cfg) {
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(out_cfg, &s_current_cfg, sizeof(s_current_cfg));
    return ESP_OK;
}

esp_err_t wifi_manager_set_config(const nvs_wifi_config_t *new_cfg)
{
    if (!new_cfg) {
        return ESP_ERR_INVALID_ARG;
    }

    // Zapis do zmiennej globalnej w RAM
    memcpy(&s_current_cfg, new_cfg, sizeof(s_current_cfg));

    // Zapis do NVS
    esp_err_t err = nvs_manager_set_wifi_config(new_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving config to NVS: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Saved new Wi-Fi config. Restarting into STA...");

    // Najprościej: restart urządzenia, aby "czyściutko" się przełączyć.
    // Można zrobić to także bez restartu (esp_wifi_stop(), zmiana mode itp.).
    esp_restart();

    return ESP_OK;
}


