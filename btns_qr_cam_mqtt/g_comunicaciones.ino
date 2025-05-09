void suscribirseATopics() {
  
  // TODO: añadir suscripciones a los topics MQTT ...
  mqtt_subscribe(CAM_TOPIC);
  mqtt_subscribe(RDK_TOPIC);

}

void alRecibirMensajePorTopic(char* topic, String incomingMessage) {


}

void enviarMensajePorTopic(const char* topic, String acc, String pers) {
  StaticJsonDocument<128> doc;

  doc["acción"] = acc;
  doc["persona"] = pers;


  char jsonBuffer[128];
  serializeJson(doc, jsonBuffer);

  mqtt_publish(topic, jsonBuffer);

}





