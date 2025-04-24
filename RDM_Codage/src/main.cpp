// Bibliothèque
#include <Arduino.h>
#include <Wire.h>     // pour l'oled hd44780
#include <hd44780.h>  // pour l'oled hd44780
#include <hd44780ioClass/hd44780_I2Cexp.h> // pour l'oled hd44780
#include <AiEsp32RotaryEncoder.h> // pour l'encodeur rotatif, teste de simulation
#include <DHT.h> // pour le dht22

// Définitions des broches
#define PRESSURE_PIN 25     // Capteur de pression
#define BUTTON_PIN    13    // Bouton pour envoyer les données du dht22 & du capteur de pression
#define LED_GREEN_PIN   15    // LED verte d'état ON
#define LED_RED_PIN 2     // LED rouge d'état OFF
#define DHT_PIN       26    // DHT22 pour l'humidité & la température
#define DHTTYPE       DHT22 // Type de capteur

// RGB pour signaler l'état du capteur de pression
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

int encoder_value = 0;

int last_encodeur_value = 0;
int last_capteur_value = 0;
int btn_flag = 0;
float poidsKg = 0;
int previousButtonState = HIGH; // bouton non appuyé initialement
int flag_confirm = 0; // flag pour activer le traitement après confirmation depuis LabVIEW

//========================================================================================================================//
// Code pour simuler le capteur de pression, encodeur rotatif qu'on fait tourner
#define ROTARY_ENCODER_A_PIN 32
#define ROTARY_ENCODER_B_PIN 33
#define ROTARY_ENCODER_BUTTON_PIN 35
#define ROTARY_ENCODER_VCC_PIN -1
#define ROTARY_ENCODER_STEPS 4

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, 
ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

int last_value_encodeur = 0;
int lastValue = -1;

void rotary_onButtonClick() {
  static unsigned long lastTimePressed = 0;
  if (millis() - lastTimePressed < 500) {
    return;
  }
  lastTimePressed = millis();
  
  delay(100);
}

void rotary_loop() {
  if (rotaryEncoder.encoderChanged()) {
    int value = rotaryEncoder.readEncoder();
    
    if (value != lastValue) {
      lastValue = value;
      encoder_value = value; // ← Mise à jour de la variable globale
    }
  }
  
  if (rotaryEncoder.isEncoderButtonClicked()) {
    rotary_onButtonClick();
  }
}


void IRAM_ATTR readEncoderISR() {
  rotaryEncoder.readEncoder_ISR();
}
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


  
  lcd.setCursor(0, 0);
  lcd.print("T:      C Kg:     ");
  lcd.setCursor(0, 1);
  lcd.print("H:      % ADC:    ");


  dht.begin();

  // digitalWrite(LED_GREEN_PIN, LOW);
  // digitalWrite(LED_RED_PIN, HIGH);

    // LED RGB blanche au démarrage
  ledcWrite(redChannel, 30);
  ledcWrite(greenChannel, 30);
  ledcWrite(blueChannel, 30);


//========================================================================================================================//
//Encodeur rotatif simulation
rotaryEncoder.begin();
rotaryEncoder.setup(readEncoderISR);
bool circleValues = false;
rotaryEncoder.setBoundaries(0, 4095, circleValues);
rotaryEncoder.setAcceleration(300);
//========================================================================================================================//
}

void loop() {
  // Lecture capteurs
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  int pressureValue = analogRead(PRESSURE_PIN);

  rotary_loop();

  // Vérifie le capteur DHT22
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Erreur de lecture du DHT22 !");
    delay(500);
    return;
  }

  if (Serial.available()) {
    String message = Serial.readStringUntil(';'); // lit le message jusqu'à ";"
  
    if (message == "confirmation") {
      Serial.println("Message de confirmation acquit");
      flag_confirm = 1; // active le traitement
      btn_flag = 0;  // réinitialise le bouton
    }
  
    if (message == "termine") {
      Serial.println("Fin de transmission");
      flag_confirm = 0; // désactive le traitement
      btn_flag = 0;  // empêche tout envoi de données
      // Revenir à une couleur blanche ou éteindre la LED
      ledcWrite(redChannel, 255);
      ledcWrite(greenChannel, 255);
      ledcWrite(blueChannel, 255);
    }
  }
  

  // gestion du bouton physique uniquement si LabVIEW a envoyé "confirmation"
  if (flag_confirm == 1) {
    int currentButtonState = digitalRead(BUTTON_PIN);
    if (previousButtonState == HIGH && currentButtonState == LOW) {
      btn_flag = !btn_flag; // bascule de 0 à 1 ou 1 à 0
    }
    previousButtonState = currentButtonState;
  }

  // // Hystérésis du capteur de pression
  // if (!(pressureValue >= last_capteur_value - 5 && pressureValue <= last_capteur_value + 5)) {
  //   last_capteur_value = pressureValue;
  // }

  // // Conversion ADC -> Kilogrammes
  // if (pressureValue >= 11) {
  //   poidsKg = 0.4028 * pressureValue + 17.57;
  // } else {
  //   poidsKg = 0;
  // }

  // // LED RGB en fonction du niveau de pression
  // if (pressureValue <= 819) {
  //   ledcWrite(redChannel, 0); ledcWrite(greenChannel, 255); ledcWrite(blueChannel, 0);
  // } else if (pressureValue <= 1638) {
  //   ledcWrite(redChannel, 127); ledcWrite(greenChannel, 255); ledcWrite(blueChannel, 0);
  // } else if (pressureValue <= 2457) {
  //   ledcWrite(redChannel, 255); ledcWrite(greenChannel, 255); ledcWrite(blueChannel, 0);
  // } else if (pressureValue <= 3276) {
  //   ledcWrite(redChannel, 255); ledcWrite(greenChannel, 128); ledcWrite(blueChannel, 0);
  // } else if (pressureValue <= 3956) {
  //   ledcWrite(redChannel, 255); ledcWrite(greenChannel, 0); ledcWrite(blueChannel, 0);
  // } else if (pressureValue <= 4000) {
  //   ledcWrite(redChannel, 0); ledcWrite(greenChannel, 0); ledcWrite(blueChannel, 255);
  // } else if (pressureValue > 4000) {
  //   ledcWrite(redChannel, 200); ledcWrite(greenChannel, 0); ledcWrite(blueChannel, 200);
  // }

//========================================================================================================================//
  // Hystérésis du capteur de pression
  if (!(encoder_value >= last_encodeur_value - 5 && pressureValue <= last_encodeur_value + 5)) {
    last_encodeur_value = pressureValue;
  }

  // Conversion ADC -> Kilogrammes
  if (encoder_value >= 11) {
    poidsKg = 0.4028 * encoder_value + 17.57;
  } else {
    poidsKg = 0;
  }

  // LED RGB en fonction du niveau de pression
  if (flag_confirm == 1) {
    if (encoder_value <= 819) {
      ledcWrite(redChannel, 0); ledcWrite(greenChannel, 200); ledcWrite(blueChannel, 0);
    } else if (encoder_value <= 1638) {
      ledcWrite(redChannel, 100); ledcWrite(greenChannel, 200); ledcWrite(blueChannel, 0);
    } else if (encoder_value <= 2457) {
      ledcWrite(redChannel, 200); ledcWrite(greenChannel, 200); ledcWrite(blueChannel, 0);
    } else if (encoder_value <= 3276) {
      ledcWrite(redChannel, 200); ledcWrite(greenChannel, 100); ledcWrite(blueChannel, 0);
    } else if (encoder_value <= 3956) {
      ledcWrite(redChannel, 200); ledcWrite(greenChannel, 0); ledcWrite(blueChannel, 0);
    } else if (encoder_value <= 4000) {
      ledcWrite(redChannel, 0); ledcWrite(greenChannel, 0); ledcWrite(blueChannel, 200);
    } else if (encoder_value > 4000) {
      ledcWrite(redChannel, 150); ledcWrite(greenChannel, 0); ledcWrite(blueChannel, 150);
    }
  }
  
//========================================================================================================================//

    // Mise à jour des LED d'état ON/OFF
  if (flag_confirm == 1) {
    if (btn_flag == 1) {
      digitalWrite(LED_GREEN_PIN, HIGH);
      digitalWrite(LED_RED_PIN, LOW);
    } else {
      digitalWrite(LED_GREEN_PIN, LOW);
      digitalWrite(LED_RED_PIN, HIGH);
    }
  } else {
    // Si on n'est pas en mode actif, toute les leds s'éteint
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_RED_PIN, LOW);
  }

  // Affichage LCD dynamique
  if (flag_confirm == 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("En attente de");
    lcd.setCursor(0, 1);
    lcd.print("confirmation...");
  }
  else if (flag_confirm == 1 && btn_flag == 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T:"); lcd.print(temperature, 1);
    lcd.print("C H:"); lcd.print(humidity, 1); lcd.print("%");

    lcd.setCursor(0, 1);
    lcd.print("Kg:"); lcd.print(poidsKg, 1);
    lcd.print(" ADC:"); lcd.print(encoder_value);
  }
  else if (flag_confirm == 1 && btn_flag == 1) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(">> Envoi des");
    lcd.setCursor(0, 1);
    lcd.print(">> donnees...");
  }
  else if (flag_confirm == -1) {  // signal de "termine"
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(">> FIN <<");
    lcd.setCursor(0, 1);
    lcd.print("                ");
  }

  

  // Envoi des données si le flag est activé
  if (btn_flag == 1) {
    Serial.print(temperature);
    Serial.print(";");
    Serial.print(humidity);
    Serial.print(";");
    Serial.print(poidsKg);
    Serial.print(";");
  }

  delay(500); // pour lisibilité
}
