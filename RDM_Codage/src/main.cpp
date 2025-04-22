// Bibliothèque
#include <Arduino.h>
#include <Wire.h>     //pour l'oled hd44780
#include <hd44780.h>  //pour l'oled hd44780
#include <hd44780ioClass/hd44780_I2Cexp.h> //pour l'oled hd44780
#include <AiEsp32RotaryEncoder.h> //pour l'encodeur rotatif, teste de simulation
#include <DHT.h> //pour le dht22

// Définitions des broches
#define PRESSURE_PIN 25     // Capteur de pression
#define BUTTON_PIN    13    // Bouton pour envoyer les données du dht22 & du capteur de pression
#define LED_RED_PIN   15    // LED rouge d'état OFF
#define LED_GREEN_PIN 2     // LED verte d'état ON
#define DHT_PIN       26    // DHT22 pour l'humidité & la température
#define DHTTYPE       DHT22 // Type de capteur

//RGB pour signaler l'état du capteur de pression
#define RED_PIN   27    
#define GREEN_PIN 14     
#define BLUE_PIN  12        
// Canaux Leds (PWM)
const int redChannel   = 0;
const int greenChannel = 1;
const int blueChannel  = 2;
// Paramètres PWM
const int freq = 5000;      // Fréquence en Hz
const int resolution = 8;   // Résolution (8 bits : valeurs de 0 à 255)

// Initialisation de l'écran LCD & du DHT
hd44780_I2Cexp lcd(0x27);
DHT dht(DHT_PIN, DHTTYPE);


int last_capteur_value = 0;
int btn_flag = 0;
float poidsKg = 0;
int previousButtonState = HIGH; // bouton non appuyé initialement

//========================================================================================================================//
//Code pour simuler le capteur de pression, encodeur rotatif qu'on fait tourner
// #define ROTARY_ENCODER_A_PIN 32
// #define ROTARY_ENCODER_B_PIN 33
// #define ROTARY_ENCODER_BUTTON_PIN 35
// #define ROTARY_ENCODER_VCC_PIN -1

// #define ROTARY_ENCODER_STEPS 4

// AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, 
// ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

// int last_value_encodeur = 0;
// int lastValue = -1;

// void rotary_onButtonClick() {
//   static unsigned long lastTimePressed = 0;
//   if (millis() - lastTimePressed < 500) {
//     return;
//   }
//   lastTimePressed = millis();
  
//   // Affichage d'un message lors du clic sur le bouton
//   lcd.clear();
//   lcd.setCursor(0, 0);
//   lcd.print("Bouton presse!");
//   delay(100);
// }

// void rotary_loop() {
//   if (rotaryEncoder.encoderChanged()) {
//     int value = rotaryEncoder.readEncoder();
    
//     if (value != lastValue) {
//       lastValue = value;
      
//       // Envoi des données au format "P:XXXX;"
//       Serial.print(value);
//       Serial.println(";");
      
//     }
//   }
  
//   if (rotaryEncoder.isEncoderButtonClicked()) {
//     rotary_onButtonClick();
//   }
// }

// void IRAM_ATTR readEncoderISR() {
//   rotaryEncoder.readEncoder_ISR();
// }
//========================================================================================================================//


void setup() {
  Serial.begin(115200);
  Wire.begin();
  lcd.begin(16, 2);       // écran 16 colonnes x 2 lignes
  lcd.setBacklight(255);  // mettre le rétroéclairage au maximum
  
  // Configuration des broches
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  
  // Initialisation des canaux PWM pour chaque couleur
  ledcSetup(redChannel, freq, resolution);
  ledcSetup(greenChannel, freq, resolution);
  ledcSetup(blueChannel, freq, resolution);

  // Attachement des broches aux canaux correspondants
  ledcAttachPin(RED_PIN, redChannel);
  ledcAttachPin(GREEN_PIN, greenChannel);
  ledcAttachPin(BLUE_PIN, blueChannel);

  dht.begin();

  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, HIGH);

//========================================================================================================================//
  //Encodeur rotatif simulation
  // rotaryEncoder.begin();
  // rotaryEncoder.setup(readEncoderISR);
  
  // bool circleValues = false;
  // rotaryEncoder.setBoundaries(0, 4095, circleValues);
  // rotaryEncoder.setAcceleration(300);
//========================================================================================================================//

}

// Ma boucle while
void loop() {
  //reçoit un string "confirmation" depuis le bouton LabView
  if (Serial.available()) {
    String message = Serial.readStringUntil(';'); //lis le message jusqu'à ";"
    if (message == "confirmation") {              //condition si le message = confirmation
      Serial.println("Message reçu !");
      // Lecture des données du capteur DHT22
      float temperature = dht.readTemperature();
      float humidity = dht.readHumidity();
      int pressureValue = analogRead(25);
    // Vérifie le capteur DHT22
    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("Erreur de lecture du DHT22 !");
      delay(500);
      return;
    }
    int currentButtonState = digitalRead(BUTTON_PIN);
    if (previousButtonState == HIGH && currentButtonState == LOW) {
      // Front descendant détecté : bouton pressé
      btn_flag = !btn_flag; // bascule de 0 à 1 ou 1 à 0
      Serial.print("Flag modifié: ");
      Serial.println(btn_flag);
      // Indication visuelle
      if (btn_flag == 1) {
        digitalWrite(LED_GREEN_PIN, HIGH);
        digitalWrite(LED_RED_PIN, LOW);
      } else {
        digitalWrite(LED_GREEN_PIN, LOW);
        digitalWrite(LED_RED_PIN, HIGH);
      }
      delay(50); // anti-rebond simple
    }

        previousButtonState = currentButtonState;

      // Hystérésis avec l’encodeur simulant le capteur de pression
      if (!(pressureValue >= last_capteur_value - 5 && pressureValue <= last_capteur_value + 5)) {
        Serial.print("Valeur Capteur (ADC): ");
        Serial.println(last_capteur_value);
        last_capteur_value = pressureValue;
      }  

      // === Conversion ADC -> Kilogrammes ===
    if (pressureValue >= 11) {                    //refaire l'étalonnage et calcul de la pente 
      poidsKg = 0.4028 * pressureValue + 17.57;
      Serial.print("Kg: ");
      Serial.print(poidsKg);
    } else {
      poidsKg = 0;
    }
      
      if (pressureValue <= 819) {               //vert foncé pas de problème
        ledcWrite(redChannel,   0);
        ledcWrite(greenChannel, 255);
        ledcWrite(blueChannel,  0);
      } else if (pressureValue <= 1638) {       // Vert clair pas de problème
        ledcWrite(redChannel,   127);
        ledcWrite(greenChannel, 255);
        ledcWrite(blueChannel,  0);
      } else if (pressureValue <= 2457) {       // Jaune on est à 50% de la plage du capteur
        ledcWrite(redChannel,   255);
        ledcWrite(greenChannel, 255);
        ledcWrite(blueChannel,  0);
      } else if (pressureValue <= 3276) {       // Orange on est vers 75% de la plage du capteur
        ledcWrite(redChannel,   255);
        ledcWrite(greenChannel, 128);
        ledcWrite(blueChannel,  0);
      } else if (pressureValue <= 3956) {        // Rouge on est presque à la limite de la plage
        ledcWrite(redChannel,   255);
        ledcWrite(greenChannel, 0);
        ledcWrite(blueChannel,  0);
      } else if (pressureValue <= 4000) {        // Bleu on va casser le capteur si on va plus loin
        ledcWrite(redChannel,   0);
        ledcWrite(greenChannel, 0);
        ledcWrite(blueChannel,  255);
      } else if (pressureValue > 4000) {        // Mauve STOOOOOP
        ledcWrite(redChannel,   200);
        ledcWrite(greenChannel, 0);
        ledcWrite(blueChannel,  200);
      }
      
      if (btn_flag == 1) {
        Serial.print(temperature);
        Serial.print(";");
        Serial.print(humidity);
        Serial.print(";");
        Serial.print(poidsKg);
        Serial.println(";");
      }
      // Affichage sur l'écran LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Temp:");
      lcd.print(temperature);
      lcd.print("C");
      lcd.print(" Kg:");
      lcd.setCursor(0, 1);
      lcd.print("Hum:");
      lcd.print(humidity);
      lcd.print("%  ");    
      lcd.print(poidsKg); 
      delay(100);

  }
}
//========================================================================================================================//
  // Lecture de la valeur de l'encodeur pour simuler la pression
  // int value = rotaryEncoder.readEncoder();

  // // Hystérésis avec l’encodeur simulant le capteur de pression
  // if (!(value >= last_value_encodeur - 5 && value <= last_value_encodeur + 5)) {
  //   Serial.print("Valeur simulée (encodeur) : ");
  //   Serial.println(last_value_encodeur);
  //   last_value_encodeur = value;
  // }

  // rotary_loop();
  // delay(100);
//========================================================================================================================//

}