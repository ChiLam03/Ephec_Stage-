// Bibliothèque
#include <Arduino.h>
#include <Wire.h>     // LCD hd44780
#include <hd44780.h>  // LCD hd44780
#include <hd44780ioClass/hd44780_I2Cexp.h> // LCD hd44780
#include <DHT.h> // DHT22
#include <Adafruit_NeoPixel.h> // Bande RGB

// Définitions des broches
#define PRESSURE_PIN 25     // Capteur de pression
#define BUTTON_PIN    13    // Bouton
#define LED_GREEN_PIN   2    // LED verte
#define LED_RED_PIN 15     // LED rouge
#define DHT_PIN       26    // DHT22 pour la température et l'humidité
#define DHTTYPE       DHT22 // Type du capteur
#define PIN_RGB        33     // GPIO pour la bande RGB
#define NUM_PIXELS     90    // Nombre de led
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
const int freq = 2000;      // Fréquence en Hz
const int resolution = 8;   // Résolution (8 bits : valeurs de 0 à 255)

// Initialisation de l'écran LCD & du DHT
hd44780_I2Cexp lcd(0x27);
DHT dht(DHT_PIN, DHTTYPE);

// Variables globales
int last_capteur_value = 0;
int previousButtonState = HIGH;
bool envoiAuto = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  lcd.begin(16, 2);       // écran 16 colonnes x 2 lignes
  lcd.setBacklight(255);  // mettre le rétroéclairage au maximum
  strip.begin();             // Initialise le ruban
  strip.show();              // Éteint toutes les LEDs
  strip.setBrightness(50);   // Ajuste la luminosité (0 à 255)

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

  // LED rouge allumée au départ
  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_GREEN_PIN, LOW);
}

void loop() {
  // Lecture des capteurs
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  last_capteur_value = analogRead(PRESSURE_PIN);
  // Affiche sur la ligne 1 les valeurs de l'ADC
  lcd.setCursor(0, 1);
  lcd.print("                "); // Efface ligne 1 avec des espaces vides pour éviter de tout clear.
  lcd.setCursor(0,1);
  lcd.print("ADC:");
  lcd.print(last_capteur_value);

  // Gestion d'erreur pour le DHT22
  if (isnan(temperature) || isnan(humidity)) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Erreur DHT22");
    delay(500);
    return;
  }

  // Lecture du buffer (Attende d'un message avec ',')
  if (Serial.available()) {
    String message = Serial.readStringUntil('\n'); // Lit tous les caractères jusqu'à un retour chariot
    message.trim();                                // Enlève les espaces ou retour chariot
    if (message.indexOf(',') != -1) {              // Si le message contient une virgule
      envoiAuto = false;                           // Passe la variable envoiAuto en False
      Serial.print(temperature);                   // Envoi les données de la Temp;Hum;ADC;message;
      Serial.print(";");                           // Et on éteint la led rouge pour allumer la led verte
      Serial.print(humidity);
      Serial.print(";");
      Serial.print(last_capteur_value);
      Serial.print(";");
      Serial.print(message);
      Serial.println(";");
      digitalWrite(LED_RED_PIN, LOW);
      digitalWrite(LED_GREEN_PIN, HIGH);
    }
  }

  // État de la led RGB & de la bande RGB (j'ai BGR) selon la pression
  if (last_capteur_value <= 819) {
    ledcWrite(redChannel, 0); ledcWrite(greenChannel, 255); ledcWrite(blueChannel, 0);   //couleur VERT
    for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(0, 255, 0));
  } else if (last_capteur_value <= 1638) {
    ledcWrite(redChannel, 127); ledcWrite(greenChannel, 255); ledcWrite(blueChannel, 0);   //couleur JAUNE-VERT
    for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(0, 255, 127));
  } else if (last_capteur_value <= 2457) {
    ledcWrite(redChannel, 255); ledcWrite(greenChannel, 255); ledcWrite(blueChannel, 0);   //couleur JAUNE
    for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(0, 255, 255));
  } else if (last_capteur_value <= 3276) {
    ledcWrite(redChannel, 255); ledcWrite(greenChannel, 128); ledcWrite(blueChannel, 0);   //couleur ORANGE
    for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(0, 128, 255));
  } else if (last_capteur_value <= 3956) {
    ledcWrite(redChannel, 255); ledcWrite(greenChannel, 0); ledcWrite(blueChannel, 0);   //couleur ROUGE
    for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(255, 0, 0));
  } else {
    ledcWrite(redChannel, 120); ledcWrite(greenChannel, 0); ledcWrite(blueChannel, 255);   //couleur BLEU-VIOLET
    for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(200, 0, 255));
  }
  strip.show();

  // Affichage du LCD sur la ligne 0 (T:xx.xC H:xx.x%)
  lcd.setCursor(0, 0);
  lcd.print("T:"); 
  lcd.print(temperature, 1); //affiche la température avec un chiffre après la virgule
  lcd.print("C H:"); 
  lcd.print(humidity, 1);    //affiche l'humidité avec un chiffre après la virgule
  lcd.print("%");

  // Gestion du bouton pour stop l'acquisition et envoyer le DHT22 en continu
  int currentButtonState = digitalRead(BUTTON_PIN); //lecture du bouton
  if (previousButtonState == HIGH && currentButtonState == LOW) { //Condition de l'état du bouton
    // Début envoi automatique
    envoiAuto = true; // Passe la variable envoiAuto en True
    digitalWrite(LED_RED_PIN, HIGH); //Led rouge allumé
    digitalWrite(LED_GREEN_PIN, LOW); //Led verte éteinte
  }
  previousButtonState = currentButtonState; //mémorise l'état du bouton

    // Envoi automatiquement quand envoiAuto est True
  if (envoiAuto) {
    Serial.print(temperature);
    Serial.print(";");
    Serial.print(humidity);
    Serial.print(";");
  }
  delay(200); //délai 200ms
}
