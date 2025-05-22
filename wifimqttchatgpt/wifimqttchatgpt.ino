#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "esp_camera.h"
#include "src/quirc/quirc.h"
#include <Wire.h>
#include <LCD_I2C.h>

// ---- CONFIGURACIÓN WIFI Y MQTT ----
const char* ssid = "Phone_1_2911";
const char* password = "11111111";
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

// ---- TOPICS MQTT ----
const char* topic_botones = "PR2/A9/RoboDK/Acciones";
const char* topic_qr = "PR2/A9/RoboDK/QR";
const char* topic_th = "PR2/A9/NT";

// ---- PINES BOTONES ----
#define BOTON1 19
#define BOTON2 20
#define BOTON3 21

// ---- CÁMARA y QR ----
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM     4
#define SIOC_GPIO_NUM     5
#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM       8
#define Y3_GPIO_NUM       9
#define Y2_GPIO_NUM       11
#define VSYNC_GPIO_NUM    6
#define HREF_GPIO_NUM     7
#define PCLK_GPIO_NUM     13

//variables globales
String persona = "Desconocido";
volatile int temp = 25;
volatile int hum = 67;
bool qr_leido = false;
int id = 1;
LCD_I2C lcd(0x3F, 16, 2);  

struct quirc *q = NULL;
uint8_t *image = NULL;  
camera_fb_t * fb = NULL;
struct quirc_code code;
struct quirc_data data;

// ---- FUNCIONES JSON POR MQTT ----
void enviarJSON(const char* topic, const char* accion, const char* persona) {
  StaticJsonDocument<128> doc;
  doc["accion"] = accion;
  doc["persona"] = persona;
  char buffer[128];
  serializeJson(doc, buffer);
  client.publish(topic, buffer);
  Serial.printf("📤 JSON publicado a [%s]: %s\n", topic, buffer);
}

void callback_topic(char* topic, byte* payload, unsigned int length) {
  String mensaje;
  for (unsigned int i = 0; i < length; i++) {
    mensaje += (char)payload[i];
  }
  Serial.print("Mensaje recibido en topic: ");
  Serial.println(topic);
  Serial.print("Contenido: ");
  Serial.println(mensaje);

  // Procesar el topic de temperatura y humedad
  if (String(topic) == topic_th) {
    lcd.clear();
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, mensaje);
    if (!error) {
      if (doc.containsKey("temperatura") && doc.containsKey("humedad")) {
        temp = doc["temperatura"];
        hum = doc["humedad"];
        Serial.printf("Temperatura y humedad actualizadas: %d C, %d %%\n", temp, hum);
      } else {
        Serial.println("JSON recibido no tiene las claves 'temperatura' o 'humedad'");
      }
    } else {
      Serial.print("Error al parsear JSON en topic_th: ");
      Serial.println(error.c_str());
    }
  }
}

// ---- WIFI Y MQTT ----
void conectarWiFi() {
  Serial.print("Conectando a WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("✅ Conectado");
  Serial.print("IP local: "); Serial.println(WiFi.localIP());
}

void conectarMQTT() {
  while (!client.connected()) {
    Serial.print("Conectando al broker MQTT...");
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("✅ Conectado");
      client.subscribe(topic_th);  
    } else {
      Serial.print("❌ Fallo. Estado: "); Serial.println(client.state());
      delay(2000);
    }
  }
}

void dumpData_bis(const struct quirc_data *data) {
  String qrTexto = (const char*)data->payload;
  Serial.print("📷 QR Leído: "); Serial.println(qrTexto);

  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, qrTexto);


  if (error) {
    Serial.println("❌ Error al parsear JSON");
    lcd.print("Acceso denegado");
    return;
  }

  const char* nombre = doc["nombre"];
  id = doc["id"];
  persona = nombre;
  client.publish(topic_qr, qrTexto.c_str());

}

struct Button {
  const uint8_t PIN;
  bool pressed;
};

Button button1 = {BOTON1, false};
Button button2 = {BOTON2, false};
Button button3 = {BOTON3, false};

void IRAM_ATTR ISR_Boton1() { button1.pressed = true; }
void IRAM_ATTR ISR_Boton2() { button2.pressed = true; }
void IRAM_ATTR ISR_Boton3() { button3.pressed = true; }

void QRCodeReader(void * args) {
  QueueHandle_t cola = (QueueHandle_t)args;

  while (true) {
    q = quirc_new();
    if (!q) continue;

    fb = esp_camera_fb_get();
    if (!fb) continue;

    quirc_resize(q, fb->width, fb->height);
    image = quirc_begin(q, NULL, NULL);
    memcpy(image, fb->buf, fb->len);
    quirc_end(q);

    int count = quirc_count(q);
    if (count > 0) {
      quirc_extract(q, 0, &code);
      if (quirc_decode(&code, &data) == 0) {
        dumpData_bis(&data);
        char mensaje_lcd[32];
        if (id >= 1 && id <= 4) {
          snprintf(mensaje_lcd, sizeof(mensaje_lcd), "Bienvenido:|%s", persona.c_str());
        } else {
          snprintf(mensaje_lcd, sizeof(mensaje_lcd), "Acceso denegado|");
        }
        xQueueSendToBack(cola, &mensaje_lcd, pdMS_TO_TICKS(3000));
      }
    }

    esp_camera_fb_return(fb);
    quirc_destroy(q);
    delay(2000); // lectura cada 2 segundos
  }
}

/* ================Tareas que gestionan nuevas entradas de informacíon================*/
void controlador(void *args){
  QueueHandle_t cola = (QueueHandle_t)args;

  for (;;){
    if (button1.pressed) {
      enviarJSON(topic_botones, "marcha", persona.c_str());
      char mensaje[32] = "MARCHA";
      xQueueSendToBack(cola, &mensaje, pdMS_TO_TICKS(3000));
      button1.pressed = false;
    }

    if (button2.pressed) {
      enviarJSON(topic_botones, "paro", persona.c_str());
      char mensaje[32] = "PARO";
      xQueueSendToBack(cola, &mensaje, pdMS_TO_TICKS(3000));
      button2.pressed = false;
    }
    if (button3.pressed) {
      enviarJSON(topic_botones, "reset", persona.c_str());
      char mensaje[32] = "RESET";
      xQueueSendToBack(cola, &mensaje, pdMS_TO_TICKS(3000));
      button3.pressed = false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // evita saturar la CPU
  }
}

void display(void *args) {
  QueueHandle_t cola = (QueueHandle_t)args;
  char mensaje[32];

  ///lcd
  Wire.begin(45, 48);
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);

  for (;;) {
    if (xQueueReceive(cola, &mensaje, pdMS_TO_TICKS(1000))) {
      lcd.clear();

      // Buscar el separador '|'
      char *separador = strchr(mensaje, '|');
      if (separador != NULL) {
        *separador = '\0';  // Termina la primera línea
        lcd.setCursor(0, 0);
        lcd.print(mensaje);
        lcd.setCursor(0, 1);
        lcd.print(separador + 1);  // Lo que está después del '|'
        vTaskDelay(pdMS_TO_TICKS(4000));
      } else {
        lcd.setCursor(0, 0);
        lcd.print(mensaje);
        vTaskDelay(pdMS_TO_TICKS(2000));
      }
      
      lcd.clear();
    } else {
      lcd.setCursor(0, 0);
      lcd.print("Tem: ");
      lcd.print(temp);
      lcd.print(" C");
      lcd.setCursor(0, 1);
      lcd.print("Hum: ");
      lcd.print(hum);
      lcd.print(" %");
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}


void setup() {
  Serial.begin(115200);

  //Botones
  pinMode(BOTON1, INPUT_PULLUP);
  pinMode(BOTON2, INPUT_PULLUP);
  pinMode(BOTON3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BOTON1), ISR_Boton1, FALLING);
  attachInterrupt(digitalPinToInterrupt(BOTON2), ISR_Boton2, FALLING);
  attachInterrupt(digitalPinToInterrupt(BOTON3), ISR_Boton3, FALLING);

  // Config cámara
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
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("❌ Error al iniciar la cámara");
    ESP.restart();
  }

  conectarWiFi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback_topic); 
  conectarMQTT();

  QueueHandle_t cola = xQueueCreate(10, sizeof(char[32]));
  xTaskCreatePinnedToCore(QRCodeReader, "QRReader", 10000,  (void*)cola, 5, NULL, 0);
  // corre en el core 0 para que esté apartada de las demás tareas 
  // y que no se interrumpa el leer códigos QR
  xTaskCreatePinnedToCore(&controlador, "controlador", 8192, (void*)cola, 2, NULL, 1); 
  //tiene una prioridad más baja porque corre en el fondo
  // es la tarea que se encarga de analizar si ha habido una interruoción
  // y entonces hacer lo que le toca
  xTaskCreatePinnedToCore(&display, "lcd", 8192, (void*)cola, 5, NULL, 1);
  // está cada 3 segundos mirando si hay algo en el la cola y en caso afirmativo,
  // lo imprime en la pantalla, en otro caso imprime la temperatura y humedad


}


void loop() {
  client.loop();
}