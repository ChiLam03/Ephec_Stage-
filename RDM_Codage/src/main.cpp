// Bibliothèque
#include <Arduino.h>
#include <Wire.h>     // pour l'oled hd44780
#include <hd44780.h>  // pour l'oled hd44780
#include <hd44780ioClass/hd44780_I2Cexp.h> // pour l'oled hd44780
#include <AiEsp32RotaryEncoder.h> // pour l'encodeur rotatif, teste de simulation
#include <DHT.h> // pour le dht22
#include <Adafruit_NeoPixel.h>


// Définitions des broches
#define PRESSURE_PIN 25     // Capteur de pression
#define BUTTON_PIN    13    // Bouton pour envoyer les données du dht22 & du capteur de pression
#define LED_GREEN_PIN   2    // LED verte d'état ON
#define LED_RED_PIN 15     // LED rouge d'état OFF
#define DHT_PIN       26    // DHT22 pour l'humidité & la température
#define DHTTYPE       DHT22 // Type de capteur

#define PIN_RGB        33     // GPIO pour la DATA du ruban
#define NUM_PIXELS     90    // Nombre de LEDs dans ton bandeau
Adafruit_NeoPixel strip(NUM_PIXELS, PIN_RGB, NEO_GRB + NEO_KHZ800);

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

// Variables globales
//int encoder_value = 0;
int last_encodeur_value = 0;
int last_capteur_value = 0;
float poidsKg = 0;
int previousButtonState = HIGH; // bouton non appuyé initialement
int flag_confirm = 0; // flag pour activer le traitement après confirmation depuis LabVIEW
bool affichageMesures = false;
unsigned long confirmationTime = 0;

// Encodeur rotatif pour simuler la pression
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
//   delay(100);
// }

// void rotary_loop() {
//   if (rotaryEncoder.encoderChanged()) {
//     int value = rotaryEncoder.readEncoder();
//     if (value != lastValue) {
//       lastValue = value;
//       encoder_value = value; // Mise à jour de la variable globale
//     }
//   }
//   if (rotaryEncoder.isEncoderButtonClicked()) {
//     rotary_onButtonClick();
//   }
// }

// void IRAM_ATTR readEncoderISR() {
//   rotaryEncoder.readEncoder_ISR();
// }

void setup() {
  Serial.begin(115200);
  Wire.begin();
  lcd.begin(16, 2);       // écran 16 colonnes x 2 lignes
  lcd.setBacklight(255);  // mettre le rétroéclairage au maximum
  strip.begin();             // Initialise le ruban
  strip.show();              // Éteint toutes les LEDs
  strip.setBrightness(50);   // Ajuste la luminosité (0 à 255)

    // Affichage initial : attente de l'appui bouton
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Attente appuie btn");
    lcd.setCursor(0, 1);
    lcd.print("Press btn->Send Data");

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

  // LED RGB blanche au démarrage
  ledcWrite(redChannel, 30);
  ledcWrite(greenChannel, 30);
  ledcWrite(blueChannel, 30);

  // rotaryEncoder.begin();
  // rotaryEncoder.setup(readEncoderISR);
  // bool circleValues = false;
  // rotaryEncoder.setBoundaries(0, 4095, circleValues);
  // rotaryEncoder.setAcceleration(300);

  // LED rouge allumée au départ
  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_GREEN_PIN, LOW);
}

void rainbowCycle(uint8_t wait) {
  static uint16_t j = 0;
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.ColorHSV((i * 65536L / strip.numPixels() + j) % 65536));
  }
  strip.show();
  j += 256;
  delay(wait);
}


void loop() {

  // Lecture capteurs
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  int pressureValue = 300; //analogRead(PRESSURE_PIN);  // Reviens au capteur réel si branché

  // Vérifie le capteur DHT22
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Erreur de lecture du DHT22/");
    delay(500);
    return;
  }

  // Gestion bouton physique : toggle d’un état
  int currentButtonState = digitalRead(BUTTON_PIN);
  static bool envoiActif = false;
  
  if (previousButtonState == HIGH && currentButtonState == LOW) {
    envoiActif = !envoiActif; // inverse l’état
  
    if (envoiActif) {
      Serial.println("Send data/");
      digitalWrite(LED_GREEN_PIN, HIGH);
      digitalWrite(LED_RED_PIN, LOW);
  
      // Affichage début de l’envoi
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(">>>  Send Data  <<<");
      lcd.setCursor(0, 1);
      lcd.print("Vers LabVIEW...");
    } else {
      Serial.println("Fin de l'envoi/");
      digitalWrite(LED_GREEN_PIN, LOW);
      digitalWrite(LED_RED_PIN, HIGH);
  
      // Affichage arrêt
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(">>>  STOP Data  <<<");
      lcd.setCursor(0, 1);
      lcd.print("Press btn->Send Data");
    }
  }
  previousButtonState = currentButtonState;
  

  // Conversion pression → kg
  if (pressureValue >= 11) {
    poidsKg = 0.4028 * pressureValue + 17.57;
  } else {
    poidsKg = 0;
  }

  // LED RGB dynamique (si envoi actif)
// LED RGB dynamique (si envoi actif)
if (envoiActif) {
  // Gestion LED RGB
  if (pressureValue <= 819) {
    ledcWrite(redChannel, 0); ledcWrite(greenChannel, 255); ledcWrite(blueChannel, 0);
    for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(0, 255, 0));
  } else if (pressureValue <= 1638) {
    ledcWrite(redChannel, 127); ledcWrite(greenChannel, 255); ledcWrite(blueChannel, 0);
    for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(127, 255, 0));
  } else if (pressureValue <= 2457) {
    ledcWrite(redChannel, 255); ledcWrite(greenChannel, 255); ledcWrite(blueChannel, 0);
    for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(255, 255, 0));
  } else if (pressureValue <= 3276) {
    ledcWrite(redChannel, 255); ledcWrite(greenChannel, 128); ledcWrite(blueChannel, 0);
    for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(255, 128, 0));
  } else if (pressureValue <= 3956) {
    ledcWrite(redChannel, 255); ledcWrite(greenChannel, 0); ledcWrite(blueChannel, 0);
    for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(255, 0, 0));
  } else {
    ledcWrite(redChannel, 120); ledcWrite(greenChannel, 0); ledcWrite(blueChannel, 255);
    for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(120, 0, 255));
  }

  strip.show();

  // Affichage LCD
  lcd.setCursor(0, 0);
  lcd.print("T:"); lcd.print(temperature, 1);
  lcd.print("C H:"); lcd.print(humidity, 1); lcd.print("%");
  lcd.setCursor(0, 1);
  lcd.print("Kg:"); lcd.print(poidsKg, 1);
  lcd.print(" ADC:"); lcd.print(pressureValue);

  // Envoi série
  Serial.print(temperature); 
  Serial.print(";");
  Serial.print(humidity); 
  Serial.print(";");
  Serial.print(poidsKg); 
  Serial.println(";");

  delay(500);  // Ce delay reste seulement ici
} else {
  // Effet visuel stylé pendant l’attente
  rainbowCycle(5);  // Effet fluide et continu
}
}