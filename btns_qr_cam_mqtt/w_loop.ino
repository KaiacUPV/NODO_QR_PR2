

void on_loop() {

  String pers = "El Kaiac"; //esto luego se cambia con en base al qr
  String acc;

  if (digitalRead(BOTON1) == LOW) {
    Serial.println("Botón 1 presionado");
    acc = "Iinicio de la simulación";
    enviarMensajePorTopic(CAM_TOPIC, acc, pers);

  }
  if (digitalRead(BOTON2) == LOW) {
    Serial.println("Botón 2 presionado");
    acc = "Parada de la simulación";
    enviarMensajePorTopic(CAM_TOPIC, acc, pers);
  }
  if (digitalRead(BOTON3) == LOW) {
    Serial.println("Botón 3 presionado");
    acc = "Parada y reseteada la simulación"; 
    enviarMensajePorTopic(CAM_TOPIC, acc, pers);
  }


  }

}

