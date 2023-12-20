#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Define los pines de los sensores
const int pin_out_1 = 26;
const int pin_out_2 = 27;
const int pin_out_3 = 25;

// Define los parámetros de la red
const char *ssid = "Esp";
const char *password = "12345678";

// Define los parámetros del servidor MQTT
const char *mqtt_server = "18.210.88.162";
const int mqtt_port = 1883;
const char *mqtt_topic_1 = "topico/sensor/Espacio1";
const char *mqtt_topic_2 = "topico/sensor/Espacio2";
const char *mqtt_topic_3 = "topico/sensor/Espacio3";

// Crea el cliente MQTT
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Crea las tareas para los sensores
TaskHandle_t task_1, task_2, task_3;

// Función para leer el valor del sensor IR
int read_sensor(int pin) {
  return digitalRead(pin);
}

// Funciones de tarea para los sensores
void taskSensor1(void *pvParameters) {
  while (true) {
    int sensorValue = read_sensor(pin_out_1);
     if (sensorValue == HIGH) {
      Serial.println("S1 Disponible");
      mqttClient.publish(mqtt_topic_1, "S1 Disponible");
      } else {
    Serial.println("S1 Ocupado");
    mqttClient.publish(mqtt_topic_1, "S1 Ocupado");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

}

void taskSensor2(void *pvParameters) {
  while (true) {
    int sensorValue = read_sensor(pin_out_2);
     if (sensorValue == HIGH) {
      Serial.println("S2 Disponible");
      mqttClient.publish(mqtt_topic_2, "S2 Disponible");

      } else {
    Serial.println("S2 Ocupado");
      mqttClient.publish(mqtt_topic_2, "S2 Ocupado");

    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

}

void taskSensor3(void *pvParameters) {
  while (true) {
    int sensorValue = read_sensor(pin_out_3);
     if (sensorValue == HIGH) {
      Serial.println("S3 Disponible");
      mqttClient.publish(mqtt_topic_3, "S3 Disponible");
      } else {
    Serial.println("S3 Ocupado");
      mqttClient.publish(mqtt_topic_3, "S3 Ocupado");

    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

}

void setup() {
  Serial.begin(115200);

  // Configura los pines de los sensores
  pinMode(pin_out_1, INPUT);
  pinMode(pin_out_2, INPUT);
  pinMode(pin_out_3, INPUT);

  // Conecta a la red WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Inicia el cliente MQTT
  mqttClient.setServer(mqtt_server, mqtt_port);

  // Crea las tareas para los sensores
  xTaskCreatePinnedToCore(taskSensor1, "Task1", 10000, NULL, 1, &task_1, 0);
  xTaskCreatePinnedToCore(taskSensor2, "Task2", 10000, NULL, 1, &task_2, 0);
  xTaskCreatePinnedToCore(taskSensor3, "Task3", 10000, NULL, 1, &task_3, 0);
}

void loop() {
  // Este loop puede permanecer vacío ya que la ejecución real se realiza en las tareas.
}