String inData;

const int Input2 = 2;
const int Input3 = 3;         // <-- NUEVO INPUT AGREGADO EN PIN 3
const int Output14 = 8;

void setup() {
  Serial.begin(115200);

  // Establecer estado seguro antes de configurar como salida
   //digitalWrite(Output14, HIGH);  // HIGH = apagado (porque es activo en bajo)
  pinMode(Output14, OUTPUT);

  pinMode(Input2, INPUT_PULLUP);
  pinMode(Input3, INPUT_PULLUP);   // <-- CONFIGURACIÓN DEL NUEVO INPUT
}

void loop() {
  while (Serial.available() > 0)
  {
    char recieved = Serial.read();
    inData += recieved;

    if (recieved == '\n')
    {
      String function = inData.substring(0, 2);

      //----- JUMP IF INPUT ON -----------------------
      if (function == "JF") {
        int IJstart = inData.indexOf('X');
        int IJTabstart = inData.indexOf('T');
        int IJInputNum = inData.substring(IJstart + 1, IJTabstart).toInt();
        if (digitalRead(IJInputNum) == HIGH) {
          Serial.println("T");
        } else {
          Serial.println("F");
        }
      }

      //----- TURN OUTPUT ON (activo en bajo: LOW enciende) -----------------------
      else if (function == "ON") {
        int ONstart = inData.indexOf('X');
        int outputNum = inData.substring(ONstart + 1).toInt();
        digitalWrite(outputNum, HIGH);  // Encender (activo en bajo)
        Serial.print("Done");
      }

      //----- TURN OUTPUT OFF (activo en bajo: HIGH apaga) -----------------------
      else if (function == "OF") {
        int ONstart = inData.indexOf('X');
        int outputNum = inData.substring(ONstart + 1).toInt();
        digitalWrite(outputNum, LOW); // Apagar (activo en bajo)
        Serial.print("Done");
      }

      //----- WAIT INPUT ON -----------------------
      else if (function == "WI") {
        int WIstart = inData.indexOf('N');
        int InputNum = inData.substring(WIstart + 1).toInt();
        while (digitalRead(InputNum) == HIGH) {
          delay(5);
        }
        Serial.print("Done");
      }

      //----- WAIT INPUT OFF -----------------------
      else if (function == "WO") {
        int WIstart = inData.indexOf('N');
        int InputNum = inData.substring(WIstart + 1).toInt();
        while (digitalRead(InputNum) == LOW) {
          delay(5);
        }
        Serial.print("Done");
      }

      //----- ECHO TEST -----------------------
      else if (function == "TM") {
        String echo = inData.substring(2);
        Serial.println(echo);
      }

      inData = ""; // Limpiar buffer
    }
  }
}
