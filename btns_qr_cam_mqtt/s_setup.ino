void on_setup() {

    Serial.begin(115200);

    //Botones
    pinMode(BOTON1, INPUT_PULLUP);
    pinMode(BOTON2, INPUT_PULLUP);
    pinMode(BOTON3, INPUT_PULLUP);

    //Pantalla

     Wire.begin(10, 11);  // Pines SDA = 10, SCL = 11 para ESP32-S3
    lcd.begin();
    lcd.backlight();

    // Mostrar mensaje
    lcd.setCursor(0, 0);
    lcd.print("Bienvenido: Kai el Guapo");

    lcd.setCursor(0, 1);

}

