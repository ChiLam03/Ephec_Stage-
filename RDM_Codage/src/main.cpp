#include <Arduino.h>
#include <Wire.h>
#include <DHT.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

//DHT22
#define DHTPIN 26
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
//LCD
hd44780_I2Cexp lcd(0x27);
//Capteur de force
const int pin_cpt_force = 35;

void setup() {
  //port série à 115200 bauds
  Serial.begin(115200);
  // Initialisation I2C
  Wire.begin();
  // Initialisation DHT
  dht.begin();
  // Initialisation du LCD : 16 colonnes, 2 lignes
  lcd.begin(16, 2);
  // Allumer le rétroéclairage
  lcd.setBacklight(255);
  // Petite pause pour laisser le temps au DHT de se stabiliser
  delay(1000);
}

void loop() {
  // Lecture des données du DHT22
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  int adcValue = analogRead(pin_cpt_force);
  // Effacer l'écran LCD avant d'écrire les nouvelles valeurs
  lcd.clear();

  // Affichage ligne 1 : température
  lcd.setCursor(0, 0);
  if (isnan(temperature)) {
    lcd.print("Erreur DHT");
  } else {
    lcd.print("T:");
    lcd.print(temperature, 1); // 1 décimale pour l'affichage sur LCD
    lcd.print("C ");
    lcd.print("ADC:");
    lcd.print(adcValue);
  }

  // Affichage ligne 2 : humidité
  lcd.setCursor(0, 1);
  if (isnan(humidity)) {
    lcd.print("H: --%");
  } else {
    lcd.print("H:");
    lcd.print(humidity, 1); // 1 décimale pour l'affichage sur LCD
    lcd.print("%");
  }

  // ENVOI SUR LE PORT SERIE au format T=xx.xx;H=yy.yy;
  if (!isnan(temperature) && !isnan(humidity) && !isnan(adcValue)) {
    Serial.print("T=");
    Serial.print(temperature, 2); // 2 décimales pour la transmission
    Serial.print(";H=");
    Serial.print(humidity, 2);    // 2 décimales pour la transmission
    Serial.print(";ADC=");
    Serial.print(adcValue);
    Serial.println(";");
  } else {
    Serial.println("Erreur lecture DHT");
  }

  // Attendre 1 seconde avant la prochaine lecture
  delay(1000);
}
