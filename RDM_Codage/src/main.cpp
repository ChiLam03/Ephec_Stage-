#include <Arduino.h>
#include <Wire.h>
#include <DHT.h>
#include <hd44780.h>                     
#include <hd44780ioClass/hd44780_I2Cexp.h>  

#define DHTPIN 26
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

// Créez l'objet lcd avec l'adresse I2C
hd44780_I2Cexp lcd(0x27);

void setup() {
  Serial.begin(115200);
  dht.begin();
  Wire.begin();
  // Initialisation de l'écran LCD en mode 16 colonnes x 2 lignes
  lcd.begin(16, 2);
}

void loop() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  lcd.clear();
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Erreur de lecture du capteur DHT!");
    lcd.setCursor(0, 0);
    lcd.print("Erreur DHT!");
  } else {
    Serial.print("Température : ");
    Serial.print(temperature, 1);
    Serial.print(" C, Humidité : ");
    Serial.print(humidity, 1);
    Serial.println(" %");
    
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temperature, 1);
    lcd.print(" C");
    
    lcd.setCursor(0, 1);
    lcd.print("Hum: ");
    lcd.print(humidity, 1);
    lcd.print(" %");
  }
  
  delay(2000);  // Pause de 2 secondes entre chaque lecture
}
