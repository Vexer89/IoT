// mqtt_manager.h

#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include "mqtt_client.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// Define maximum lengths and queue sizes
#define MQTT_TOPIC_MAX_LEN 256
#define MQTT_MSG_MAX_LEN   256
#define MQTT_QUEUE_LENGTH  10

// Initialize the MQTT manager
// broker_uri: MQTT broker URI (e.g., "mqtt://192.168.144.219:1883")
// user: MQTT username
// password: MQTT password (if any)
esp_err_t init_mqtt_manager(const char *broker_uri, const char *user, const char *password);

// Send a message to a specific MQTT topic
// topic: MQTT topic to publish to
// message: Message payload
esp_err_t mqtt_manager_send(const char *topic, const char *message);

// Optionally, receive messages from a queue (if subscribing to topics)
// Returns ESP_OK on success, ESP_ERR_TIMEOUT on timeout
esp_err_t mqtt_manager_receive(char *topic, size_t topic_len, char *message, size_t message_len, TickType_t ticks_to_wait);

// Clean up the MQTT manager
void mqtt_manager_cleanup(void);

#endif // MQTT_MANAGER_H
