// mqtt_manager.c

#include "mqtt_manager.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "../nvs/nvs_manager.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

static const char *TAG = "mqtt_manager";

nvs_mqtt_config_t mqtt_custom_cfg;

// MQTT client handle
static esp_mqtt_client_handle_t mqtt_client = NULL;

// Queues for sending and receiving messages
static QueueHandle_t mqtt_send_queue = NULL;
static QueueHandle_t mqtt_receive_queue = NULL;

// Mutex for thread-safe operations
static SemaphoreHandle_t mqtt_mutex = NULL;

// Event handler for MQTT events
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

// Task to handle sending messages from the queue
static void mqtt_send_task(void *pvParameters);

// Task to handle incoming messages (if subscribed)
static void mqtt_receive_task(void *pvParameters);

// Initialize the MQTT manager
esp_err_t init_mqtt_manager(const char *broker_uri, const char *user, const char *password)
{

    // Pobranie konfiguracji z NVS
    esp_err_t err = nvs_manager_get_mqtt_config(&mqtt_custom_cfg);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "MQTT Broker: %s", mqtt_custom_cfg.mqtt_broker);
        ESP_LOGI(TAG, "MQTT Topic: %s", mqtt_custom_cfg.mqtt_topic);
    } else {
        ESP_LOGE(TAG, "Failed to load MQTT configuration from NVS");
    }
   
    // Configure MQTT client
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = broker_uri,
        .credentials.username = user,
        .credentials.authentication.password = password,
        // Optionally set other parameters like port, client_id, etc.
    };

    ESP_LOGW(TAG, "Broker: %s", mqtt_custom_cfg.mqtt_broker);
    ESP_LOGW(TAG, "Topic: %s", mqtt_custom_cfg.mqtt_topic);

    // Initialize MQTT client
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return ESP_FAIL;
    }

    // Register MQTT event handler
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

    // Create queues
    mqtt_send_queue = xQueueCreate(MQTT_QUEUE_LENGTH, sizeof(char) * (MQTT_TOPIC_MAX_LEN + MQTT_MSG_MAX_LEN));
    if (mqtt_send_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create send queue");
        return ESP_FAIL;
    }

    mqtt_receive_queue = xQueueCreate(MQTT_QUEUE_LENGTH, sizeof(char) * (MQTT_TOPIC_MAX_LEN + MQTT_MSG_MAX_LEN));
    if (mqtt_receive_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create receive queue");
        return ESP_FAIL;
    }

    // Create mutex
    mqtt_mutex = xSemaphoreCreateMutex();
    if (mqtt_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_FAIL;
    }

    // Start MQTT client
    esp_err_t start_ret = esp_mqtt_client_start(mqtt_client);
    if (start_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client");
        return start_ret;
    }

    // Create send task
    BaseType_t task_ret = xTaskCreate(mqtt_send_task, "mqtt_send_task", 4096, NULL, 5, NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create send task");
        return ESP_FAIL;
    }

    // Create receive task
    task_ret = xTaskCreate(mqtt_receive_task, "mqtt_receive_task", 4096, NULL, 5, NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create receive task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "MQTT Manager initialized successfully");
    return ESP_OK;
}

// Send a message to a specific MQTT topic
esp_err_t mqtt_manager_send(const char *topic, const char *message)
{
    if (mqtt_send_queue == NULL) {
        ESP_LOGE(TAG, "Send queue not initialized");
        return ESP_FAIL;
    }

    char full_topic[MQTT_TOPIC_MAX_LEN];
    snprintf(full_topic, sizeof(full_topic), "%s%s", mqtt_custom_cfg.mqtt_topic, topic);

    // Create a buffer to hold topic and message
    char buffer[MQTT_TOPIC_MAX_LEN + MQTT_MSG_MAX_LEN];
    memset(buffer, 0, sizeof(buffer));



    // Ensure topic and message lengths
    if (strlen(full_topic) >= MQTT_TOPIC_MAX_LEN || strlen(message) >= MQTT_MSG_MAX_LEN) {
        ESP_LOGE(TAG, "Topic or message too long");
        return ESP_ERR_INVALID_ARG;
    }

    // Combine topic and message separated by a null character
    strcpy(buffer, full_topic);
    strcpy(buffer + MQTT_TOPIC_MAX_LEN, message); // Fixed offset for simplicity

    // Send to queue
    if (xQueueSend(mqtt_send_queue, buffer, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to send to send queue");
        return ESP_FAIL;
    }

    return ESP_OK;
}

// Receive a message from the receive queue
esp_err_t mqtt_manager_receive(char *topic, size_t topic_len, char *message, size_t message_len, TickType_t ticks_to_wait)
{
    if (mqtt_receive_queue == NULL) {
        ESP_LOGE(TAG, "Receive queue not initialized");
        return ESP_FAIL;
    }

    // Buffer to receive data
    char buffer[MQTT_TOPIC_MAX_LEN + MQTT_MSG_MAX_LEN];
    memset(buffer, 0, sizeof(buffer));

    // Receive from queue
    if (xQueueReceive(mqtt_receive_queue, buffer, ticks_to_wait) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // Extract topic and message
    strncpy(topic, buffer, MQTT_TOPIC_MAX_LEN - 1);
    strncpy(message, buffer + MQTT_TOPIC_MAX_LEN, message_len - 1);

    return ESP_OK;
}

// Clean up the MQTT manager
void mqtt_manager_cleanup(void)
{
    if (mqtt_client != NULL) {
        esp_mqtt_client_stop(mqtt_client);
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
    }

    if (mqtt_send_queue != NULL) {
        vQueueDelete(mqtt_send_queue);
        mqtt_send_queue = NULL;
    }

    if (mqtt_receive_queue != NULL) {
        vQueueDelete(mqtt_receive_queue);
        mqtt_receive_queue = NULL;
    }

    if (mqtt_mutex != NULL) {
        vSemaphoreDelete(mqtt_mutex);
        mqtt_mutex = NULL;
    }

    ESP_LOGI(TAG, "MQTT Manager cleaned up");
}

// MQTT event handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        // Subscribe to topics if needed
        // Example: esp_mqtt_client_subscribe(client, "/topic/data", 0);

        char topic_smoke[128];
        char topic_temperature[128];
        char topic_reset[128];

        snprintf(topic_smoke, sizeof(topic_smoke), "%sconfig/threshold/smoke", mqtt_custom_cfg.mqtt_topic);
        snprintf(topic_temperature, sizeof(topic_temperature), "%sconfig/threshold/temperature", mqtt_custom_cfg.mqtt_topic);
        snprintf(topic_reset, sizeof(topic_reset), "%sconfig/reset", mqtt_custom_cfg.mqtt_topic);

        esp_mqtt_client_subscribe(client, topic_smoke, 0);
        esp_mqtt_client_subscribe(client, topic_temperature, 0);
        esp_mqtt_client_subscribe(client, topic_reset, 0);

        ESP_LOGI(TAG, "Subscribed to MQTT topics successfully");

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        // Handle incoming data
        // Optionally, you can push received messages to a queue
        if (mqtt_receive_queue != NULL) {
            char buffer[MQTT_TOPIC_MAX_LEN + MQTT_MSG_MAX_LEN];
            memset(buffer, 0, sizeof(buffer));

            // Copy topic
            int len = event->topic_len < MQTT_TOPIC_MAX_LEN - 1 ? event->topic_len : MQTT_TOPIC_MAX_LEN - 1;
            strncpy(buffer, event->topic, len);

            // Copy data
            len = event->data_len < MQTT_MSG_MAX_LEN - 1 ? event->data_len : MQTT_MSG_MAX_LEN - 1;
            strncpy(buffer + MQTT_TOPIC_MAX_LEN, event->data, len);

            // Send to receive queue
            if (xQueueSend(mqtt_receive_queue, buffer, portMAX_DELAY) != pdTRUE) {
                ESP_LOGE(TAG, "Failed to send to receive queue");
            }
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGE(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
            ESP_LOGE(TAG, "Reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
            ESP_LOGE(TAG, "Reported from tls stack: 0x%x", event->error_handle->esp_tls_stack_err);
            ESP_LOGE(TAG, "Captured as transport's socket errno: %d", event->error_handle->esp_transport_sock_errno);
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

// Task to handle sending messages from the queue
static void mqtt_send_task(void *pvParameters)
{
    char buffer[MQTT_TOPIC_MAX_LEN + MQTT_MSG_MAX_LEN];
    while (1) {
        if (xQueueReceive(mqtt_send_queue, buffer, portMAX_DELAY) == pdTRUE) {
            // Extract topic and message
            char topic[MQTT_TOPIC_MAX_LEN];
            char message[MQTT_MSG_MAX_LEN];
            memset(topic, 0, sizeof(topic));
            memset(message, 0, sizeof(message));

            strncpy(topic, buffer, MQTT_TOPIC_MAX_LEN - 1);
            strncpy(message, buffer + MQTT_TOPIC_MAX_LEN, MQTT_MSG_MAX_LEN - 1);

            // Publish the message
            int msg_id = esp_mqtt_client_publish(mqtt_client, topic, message, 0, 1, 0);
            ESP_LOGI(TAG, "Published to %s, msg_id=%d", topic, msg_id);
        }
    }
}

// Task to handle receiving messages from the queue (if needed)
static void mqtt_receive_task(void *pvParameters)
{
    char topic[MQTT_TOPIC_MAX_LEN];
    char message[MQTT_MSG_MAX_LEN];
    while (1) {
        if (mqtt_manager_receive(topic, sizeof(topic), message, sizeof(message), portMAX_DELAY) == ESP_OK) {
            ESP_LOGI(TAG, "Received message from topic %s: %s", topic, message);
            // Process the received message as needed
        }
    }
}
