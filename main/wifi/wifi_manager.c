#include "wifi_manager.h"
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_mac.h" // Add this line to include the correct header for ESP_MAC_WIFI_STA
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "lwip/ip4_addr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_http_server.h>
#include "cJSON.h"
#include <esp_http_client.h>
#include "../mqtt/mqtt_manager.h"

#define MQTT_BROKER_URI "mqtt://192.168.196.90:1883"
#define MQTT_USER NULL
#define MQTT_PASSWORD NULL // Replace with actual password if any
#define RESPONSE_BUFFER_SIZE 2048

// Ten plik: #include "nvs_manager.h" -> jest już w wifi_manager.h

static const char* TAG = "WIFI_MANAGER";

static nvs_wifi_config_t s_current_cfg;
static bool s_sta_connected = false;

typedef struct {
    char  *post_data;          // JSON, który wysyłamy
    char  *response_buffer;    // Bufor na odpowiedź z serwera
    int    response_buffer_len; // Rozmiar bufora na odpowiedź
    int    response_offset;    // Bieżący wskaźnik, gdzie zapisujemy kolejne dane
} http_client_data_t;


static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    http_client_data_t *client_data = (http_client_data_t *)evt->user_data;

    switch (evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGE("HTTP_EVENT", "HTTP_EVENT_ERROR");
            break;

        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI("HTTP_EVENT", "HTTP_EVENT_ON_CONNECTED");
            break;

        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGI("HTTP_EVENT", "HTTP_EVENT_HEADER_SENT");
            break;

        case HTTP_EVENT_ON_HEADER:
            // Tutaj można przechwycić nagłówki serwera (evt->header_key, evt->header_value)
            ESP_LOGD("HTTP_EVENT", "HTTP_EVENT_ON_HEADER: %s = %s",
                     evt->header_key, evt->header_value);
            break;

        case HTTP_EVENT_ON_DATA:
            // Odczyt danych przychodzących fragmentami
            if (!esp_http_client_is_chunked_response(evt->client)) {
                int copy_len = evt->data_len;
                
                // Sprawdź, ile mamy jeszcze miejsca w buforze
                int space_left = client_data->response_buffer_len - client_data->response_offset - 1;
                if (copy_len > space_left) {
                    // Jeśli dane się nie mieszczą, obcinamy
                    copy_len = space_left;
                    ESP_LOGW("HTTP_EVENT", "Bufor odpowiedzi jest za mały, przycinam dane...");
                }

                // Kopiujemy dane do bufora
                if (copy_len > 0) {
                    memcpy(client_data->response_buffer + client_data->response_offset,
                           evt->data,
                           copy_len);
                    client_data->response_offset += copy_len;
                    // Zawsze utrzymujemy '\0' na końcu
                    client_data->response_buffer[client_data->response_offset] = '\0';
                }
            }
            break;

        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI("HTTP_EVENT", "HTTP_EVENT_ON_FINISH");

            // Jeśli coś zostało odebrane, to tu mamy całą odpowiedź w response_buffer
            if (client_data->response_offset > 0) {
                ESP_LOGI("HTTP_EVENT", "Odebrana odpowiedź: %s", client_data->response_buffer);

                cJSON *response_json = cJSON_Parse(client_data->response_buffer);
                if (response_json) {
                    const cJSON *mqtt_topic = cJSON_GetObjectItem(response_json, "mqtt_topic");
                    const cJSON *mqtt_broker = cJSON_GetObjectItem(response_json, "mqttBroker");
                    if (mqtt_topic && cJSON_IsString(mqtt_topic) && 
                        mqtt_broker && cJSON_IsString(mqtt_broker)) {

                        ESP_LOGI("HTTP_EVENT", "MQTT topic: %s", mqtt_topic->valuestring);
                        ESP_LOGI("HTTP_EVENT", "MQTT broker: %s", mqtt_broker->valuestring);

                        nvs_mqtt_config_t new_cfg = {0};
                        strncpy(new_cfg.mqtt_broker, mqtt_broker->valuestring, sizeof(new_cfg.mqtt_broker) - 1);
                        strncpy(new_cfg.mqtt_topic, mqtt_topic->valuestring, sizeof(new_cfg.mqtt_topic) - 1);
                        new_cfg.mqtt_broker[sizeof(new_cfg.mqtt_broker) - 1] = '\0';
                        new_cfg.mqtt_topic[sizeof(new_cfg.mqtt_topic) - 1] = '\0';

                        esp_err_t err = nvs_manager_set_mqtt_config(&new_cfg);
                        if (err == ESP_OK) {
                            ESP_LOGI("HTTP_EVENT", "Konfiguracja MQTT zapisana poprawnie");
                        } else {
                            ESP_LOGE("HTTP_EVENT", "Błąd zapisu MQTT: %s", esp_err_to_name(err));
                        }
                    } else {
                        ESP_LOGW("HTTP_EVENT", "Brak wymaganych pól w odpowiedzi JSON lub błędny format");
                    }
                    cJSON_Delete(response_json);
                } else {
                    ESP_LOGE("HTTP_EVENT", "Nie udało się sparsować odpowiedzi JSON");
                }
            } else {
                ESP_LOGW("HTTP_EVENT", "Brak danych w odpowiedzi (lub błąd odczytu)");
            }

            break;

        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI("HTTP_EVENT", "HTTP_EVENT_DISCONNECTED");
            break;

        default:
            break;
    }
    return ESP_OK;
}


void send_pairing_request()
{
    ESP_LOGI(TAG, "Starting pairing request...");

    // 1. Pobierz konfigurację Wi-Fi
    nvs_wifi_config_t wifi_cfg;
    if (wifi_manager_get_config(&wifi_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get Wi-Fi config");
        return;
    }
    ESP_LOGI(TAG, "Wi-Fi config obtained successfully");

    // 2. Zbuduj JSON do wysłania
    cJSON *root = cJSON_CreateObject();
    if (!root) {
        ESP_LOGE(TAG, "Failed to create JSON object");
        return;
    }

    cJSON_AddStringToObject(root, "token", wifi_cfg.pairing_token);
    cJSON_AddStringToObject(root, "device_name", "Czujnik dymu");
    cJSON_AddStringToObject(root, "serial_number", "SN12345699000222");

    // 2a. Pobranie rzeczywistego adresu MAC
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    cJSON_AddStringToObject(root, "mac_address", mac_str);

    // 2b. Konwersja obiektu JSON do stringa
    char *post_data = cJSON_PrintUnformatted(root);
    cJSON_Delete(root); // Zwolnij obiekt JSON
    if (!post_data) {
        ESP_LOGE(TAG, "Failed to allocate memory for JSON data");
        return;
    }
    ESP_LOGI(TAG, "Generated JSON data: %s", post_data);

    // 3. Przygotuj strukturę na dane event handlera
    static http_client_data_t client_data;
    static char response_buffer[1024]; // Bufor na odpowiedź z serwera
    memset(&client_data, 0, sizeof(client_data));
    memset(response_buffer, 0, sizeof(response_buffer));

    client_data.post_data          = post_data;
    client_data.response_buffer    = response_buffer;
    client_data.response_buffer_len = sizeof(response_buffer);
    client_data.response_offset    = 0;

    // 4. Konfiguracja klienta HTTP (z event handlerem)
    esp_http_client_config_t config = {
        .url            = "http://192.168.196.90:3000/api/pairing/register-device",
        .method         = HTTP_METHOD_POST,
        .timeout_ms     = 15000, // 15s timeout
        .buffer_size    = 1024,
        .event_handler  = http_event_handler,  // Ustawiamy nasz event handler
        .user_data      = &client_data,        // Przekazujemy wskaźnik do naszej struktury
    };

    // 5. Inicjalizacja klienta
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        free(post_data);
        return;
    }

    // 6. Ustawienie nagłówków
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Connection", "close");

    // 7. Ustawiamy dane POST
    esp_http_client_set_post_field(client, client_data.post_data, strlen(client_data.post_data));

    // 8. Wykonujemy żądanie
    esp_err_t err = esp_http_client_perform(client);

    // 9. Spr. status kod i ew. błąd
    if (err == ESP_OK) {
        int status_code = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "HTTP request completed with status = %d", status_code);
    } else {
        ESP_LOGE(TAG, "HTTP request failed: %s", esp_err_to_name(err));
    }

    // 10. Zwolnij zasoby
    esp_http_client_cleanup(client);
    // Zwolnij post_data, jeśli już nie będzie potrzebny
    free(post_data);

    ESP_LOGI(TAG, "Pairing process completed");
}


 

/* Handler obsługujący zapis konfiguracji */
static esp_err_t save_wifi_handler(httpd_req_t *req)
{
    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive data");
        return ESP_FAIL;
    }
    content[ret] = '\0';

    char ssid[32] = {0}, pass[64] = {0}, token[128] = {0};

    // Parse the form data using ESP-IDF's query parser
    httpd_query_key_value(content, "ssid", ssid, sizeof(ssid));
    httpd_query_key_value(content, "pass", pass, sizeof(pass));
    httpd_query_key_value(content, "token", token, sizeof(token));

    ESP_LOGI(TAG, "Received SSID: %s, PASS: %s, TOKEN: %s", ssid, pass, token);

    if (strlen(ssid) == 0 || strlen(pass) == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SSID or password is missing");
        return ESP_FAIL;
    }

    // Save configuration
    nvs_wifi_config_t new_cfg = {0};
    strncpy(new_cfg.wifi_ssid, ssid, sizeof(new_cfg.wifi_ssid) - 1);
    strncpy(new_cfg.wifi_pass, pass, sizeof(new_cfg.wifi_pass) - 1);
    strncpy(new_cfg.pairing_token, token, sizeof(new_cfg.pairing_token) - 1);

    esp_err_t err = wifi_manager_set_config(&new_cfg);
    if (err == ESP_OK) {
        httpd_resp_sendstr(req, "Wi-Fi configuration saved. Restarting...");
        esp_restart();
        return ESP_OK;
    } else {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save Wi-Fi config");
        return ESP_FAIL;
    }
}

// Handler serving the HTML configuration page
static esp_err_t config_page_handler(httpd_req_t *req)
{
    const char *resp_str = "<!DOCTYPE html>"
                       "<html lang=\"en\">"
                       "<head>"
                       "<meta charset=\"UTF-8\">"
                       "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
                       "<title>Wi-Fi Configuration</title>"
                       "<style>"
                       "body {"
                       "    font-family: Arial, sans-serif;"
                       "    margin: 50px;"
                       "    background-color: #f4f4f4;"
                       "    display: flex;"
                       "    justify-content: center;"
                       "    align-items: center;"
                       "    height: 100vh;"
                       "}"
                       ".container {"
                       "    background: #fff;"
                       "    padding: 30px;"
                       "    border-radius: 10px;"
                       "    box-shadow: 0px 0px 15px rgba(0, 0, 0, 0.2);"
                       "    max-width: 400px;"
                       "    width: 100%;"
                       "}"
                       "h2 {"
                       "    text-align: center;"
                       "    color: #333;"
                       "    margin-bottom: 20px;"
                       "}"
                       "input[type='text'], input[type='password'] {"
                       "    width: 100%;"
                       "    padding: 10px;"
                       "    margin: 10px 0;"
                       "    border: 1px solid #ccc;"
                       "    border-radius: 5px;"
                       "    font-size: 16px;"
                       "}"
                       "input[type='submit'] {"
                       "    background-color: #28a745;"
                       "    color: #fff;"
                       "    border: none;"
                       "    padding: 12px 20px;"
                       "    font-size: 16px;"
                       "    border-radius: 5px;"
                       "    cursor: pointer;"
                       "    width: 100%;"
                       "}"
                       "input[type='submit']:hover {"
                       "    background-color: #218838;"
                       "}"
                       "</style>"
                       "</head>"
                       "<body>"
                       "<div class=\"container\">"
                       "    <h2>Wi-Fi Configuration</h2>"
                       "    <form action=\"/save_wifi\" method=\"POST\">"
                       "        <label for=\"ssid\">SSID:</label>"
                       "        <input type=\"text\" id=\"ssid\" name=\"ssid\" placeholder=\"Enter Wi-Fi SSID\" required><br>"
                       "        <label for=\"pass\">Password:</label>"
                       "        <input type=\"password\" id=\"pass\" name=\"pass\" placeholder=\"Enter Wi-Fi Password\" required><br>"
                       "        <label for=\"token\">Token:</label>"
                       "        <input type=\"text\" id=\"token\" name=\"token\" placeholder=\"Enter Pairing Token\" required><br>"
                       "        <input type=\"submit\" value=\"Save\">"
                       "    </form>"
                       "</div>"
                       "</body>"
                       "</html>";


    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, resp_str, strlen(resp_str));

    return ESP_OK;
}

// Function to start the web server
static httpd_handle_t start_config_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_uri_t config_page_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = config_page_handler,
            .user_ctx = NULL};

        httpd_register_uri_handler(server, &config_page_uri);

        httpd_uri_t save_wifi_uri = {
            .uri = "/save_wifi",
            .method = HTTP_POST,
            .handler = save_wifi_handler,
            .user_ctx = NULL};

        httpd_register_uri_handler(server, &save_wifi_uri);

        ESP_LOGI(TAG, "Configuration web server started");
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
                bool is_connected = false;
                esp_err_t err = nvs_manager_get_connection_flag(&is_connected);

                if (!is_connected) {
                    ESP_LOGI(TAG, "Pairing process startted.");
                    send_pairing_request();
                }
                init_mqtt_manager(MQTT_BROKER_URI, MQTT_USER, MQTT_PASSWORD);

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
            nvs_manager_set_connection_flag(false);
            memset(&s_current_cfg, 0, sizeof(s_current_cfg));
            wifi_init_ap();
            // (Opcjonalnie) start_config_server(); // prosty serwer HTTP do konfiguracji
            return ESP_OK;
        }

        // Próba odczytu z NVS
        err = nvs_manager_get_wifi_config(&s_current_cfg);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Wi-Fi Config Loaded Successfully:");
            ESP_LOGI(TAG, "SSID: %s", s_current_cfg.wifi_ssid);
            ESP_LOGI(TAG, "Password: %s", s_current_cfg.wifi_pass);
            ESP_LOGI(TAG, "Pairing Token: %s", s_current_cfg.pairing_token);
        } else {
            ESP_LOGE(TAG, "Failed to load Wi-Fi config from NVS: %s", esp_err_to_name(err));
        }

        if (err == ESP_OK && strlen(s_current_cfg.wifi_ssid) > 0) {
            // Mamy zapisane SSID – uruchamiamy STA
            ESP_LOGI(TAG, "Config found. Initialazing sta");
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

    //===========================================================================//

    


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
        //esp_wifi_stop();
        
        //wifi_init_sta(&s_current_cfg);
        
        esp_restart();
        
        return ESP_OK;
    }
    


    
