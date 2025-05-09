// COMM BAUDS
#define BAUDS 115200

#define LOGGER_ENABLED            // Comentar para deshabilitar el logger por consola serie

#define LOG_LEVEL TRACE           // nivells en c_logger: TRACE, DEBUG, INFO, WARN, ERROR, FATAL, NONE

// DEVICE
//#define DEVICE_ESP_ID             "54CE0361420"   // ESP32 ID
#define DEVICE_GIIROB_PR2_ID      "00" //"giirobpr2_00"

// WIFI
#define NET_SSID                  "UPV-PSK"
#define NET_PASSWD                "giirob-pr2-2023"

// MQTT
#define MQTT_SERVER_IP            "mqtt.dsic.upv.es"
#define MQTT_SERVER_PORT          1883
#define MQTT_USERNAME             "giirob"    // Descomentar para conexión al broker MQTT (user/passwd)
#define MQTT_PASSWORD             "UPV2024"

// SSL
// Define SSL_ROOT_CA to enable TLS/SSL connection

#define CAM_TOPIC              "PR2/A9/RoboDK"    
#define RDK_TOPIC              "PR2/A9/RoboDK"

// IO
#define BOTON1 4
#define BOTON2 5
#define BOTON3 18


