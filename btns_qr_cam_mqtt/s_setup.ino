void on_setup() {

    Serial.begin(115200);

    pinMode(BOTON1, INPUT_PULLUP);
    pinMode(BOTON2, INPUT_PULLUP);
    pinMode(BOTON3, INPUT_PULLUP);

}

