#include <Arduino.h>
#include <Wire.h>
#include <DHT.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

// Configuration du DHT22
#define DHTPIN 26
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Broche ADC de l'ESP32 pour la lecture du signal amplifié (INA125)
const int SENSOR_PIN = 35; 
const float ADC_REF_VOLTAGE = 3.3f;
const int ADC_MAX_VALUE = 4095;

// Paramètres de calibration pour le capteur de force
float tareVoltage = 0.0f;            // Tension en l'absence de charge
const float scaleFactor = 20.0f;     // Facteur de conversion (N/V), à ajuster via calibration

// Initialisation du LCD (vérifiez l'adresse I2C, ici 0x27)
hd44780_I2Cexp lcd(0x27);

// Fonction pour lire la tension sur le capteur via l'ADC
float readVoltage() {
  int rawValue = analogRead(SENSOR_PIN);
  return (ADC_REF_VOLTAGE * rawValue) / ADC_MAX_VALUE;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  dht.begin();
  lcd.begin(16, 2);
  lcd.setBacklight(255);
  
  // Attente de stabilisation des capteurs
  delay(2000);
  
  // Enregistrer la tension à vide (tare) : aucun force appliquée
  tareVoltage = readVoltage();
  Serial.print("Tare voltage (no load) = ");
  Serial.print(tareVoltage, 3);
  Serial.println(" V");
  Serial.println("Calibration terminée. Appliquez une force.");
}

void loop() {
  // Lecture des données du DHT22
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  // Lecture de la tension sur le capteur de force
  float voltage = readVoltage();
  // Calcul de la force en Newton à partir de la variation de tension
  float force = (voltage - tareVoltage) * scaleFactor;
  
  // Affichage dans le moniteur série
  Serial.print("Force (N) = ");
  Serial.print(force, 3);
  Serial.print(" | Tension = ");
  Serial.print(voltage, 3);
  Serial.println(" V");
  
  // Affichage sur le LCD (2 lignes)
  lcd.clear();
  
  // Ligne 1 : Température et Force
  lcd.setCursor(0, 0);
  if (isnan(temperature)) {
    lcd.print("Erreur DHT");
  } else {
    lcd.print("T:");
    lcd.print(temperature, 1);
    lcd.print("C ");
    lcd.print("F:");
    lcd.print(force, 1);
    lcd.print("N");
  }
  
  // Ligne 2 : Humidité
  lcd.setCursor(0, 1);
  if (isnan(humidity)) {
    lcd.print("H: --%");
  } else {
    lcd.print("H:");
    lcd.print(humidity, 1);
    lcd.print("%");
  }
  
  delay(1000);
}
