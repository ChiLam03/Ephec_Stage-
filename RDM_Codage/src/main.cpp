// Bibliothèque
#include <Arduino.h>
#include <Wire.h>     // pour l'oled hd44780
#include <hd44780.h>  // pour l'oled hd44780
#include <hd44780ioClass/hd44780_I2Cexp.h> // pour l'oled hd44780
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
const int freq = 2000;      // Fréquence en Hz
const int resolution = 8;   // Résolution (8 bits : valeurs de 0 à 255)

// Initialisation de l'écran LCD & du DHT
hd44780_I2Cexp lcd(0x27);
DHT dht(DHT_PIN, DHTTYPE);

// Variables globales
int last_encodeur_value = 0;
int last_capteur_value = 0;
float poidsKg = 0;
int previousButtonState = HIGH; // bouton non appuyé initialement
int flag_confirm = 0; // flag pour activer le traitement après confirmation depuis LabVIEW
bool affichageMesures = false;
unsigned long confirmationTime = 0;
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
  // Lecture capteurs
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  last_capteur_value = analogRead(PRESSURE_PIN);
  lcd.setCursor(0, 1);
  lcd.print("                "); // Efface ligne 1 (16 caractères)
  lcd.setCursor(0,1);
  lcd.print("ADC:");
  lcd.print(last_capteur_value);

  // Système d'erreur
  if (isnan(temperature) || isnan(humidity)) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Erreur DHT22");
    delay(500);
    return;
  }

  // Lecture série
  if (Serial.available()) {
    String message = Serial.readStringUntil('\n');
    message.trim();  // Enlève espaces ou retour chariot inutiles

    // Si le message contient une virgule
    if (message.indexOf(',') != -1) {
      envoiAuto = false; // STOP l'envoi automatique
      Serial.print(temperature);
      Serial.print(";");
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

  // État LED & bandeau RGB selon pression
  if (last_capteur_value <= 819) {
    ledcWrite(redChannel, 0); ledcWrite(greenChannel, 255); ledcWrite(blueChannel, 0);
    for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(0, 255, 0));//Bleu Vert Rouge
  } else if (last_capteur_value <= 1638) {
    ledcWrite(redChannel, 127); ledcWrite(greenChannel, 255); ledcWrite(blueChannel, 0);
    for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(0, 255, 127));
  } else if (last_capteur_value <= 2457) {
    ledcWrite(redChannel, 255); ledcWrite(greenChannel, 255); ledcWrite(blueChannel, 0);
    for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(0, 255, 255));
  } else if (last_capteur_value <= 3276) {
    ledcWrite(redChannel, 255); ledcWrite(greenChannel, 128); ledcWrite(blueChannel, 0);
    for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(0, 128, 255));
  } else if (last_capteur_value <= 3956) {
    ledcWrite(redChannel, 255); ledcWrite(greenChannel, 0); ledcWrite(blueChannel, 0);
    for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(255, 0, 0));
  } else {
    ledcWrite(redChannel, 120); ledcWrite(greenChannel, 0); ledcWrite(blueChannel, 255);
    for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(200, 0, 255));
  }

  strip.show();

  // Affichage LCD ligne 1 (température, humidité)
  lcd.setCursor(0, 0);
  lcd.print("T:"); 
  lcd.print(temperature, 1);
  lcd.print("C H:"); 
  lcd.print(humidity, 1); 
  lcd.print("%");

  int currentButtonState = digitalRead(BUTTON_PIN);
  if (previousButtonState == HIGH && currentButtonState == LOW) {
    // Début envoi automatique
    envoiAuto = true;
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_GREEN_PIN, LOW);
     
  }
  previousButtonState = currentButtonState;

    // Envoi automatique si activé
  if (envoiAuto) {
    Serial.print(temperature);
    Serial.print(";");
    Serial.print(humidity);
    Serial.print(";");
  }
  //test de bande rgb
  //for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(200, 0, 0));
  //strip.show();
  delay(200); // Délai demandé
}
