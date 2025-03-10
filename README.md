# Ephec Stage
Stagiaire (moi): TRUONG Chi Lâm  
Maître de stage: CAMMERMANS Romain
## Introduction  
Je me présente, stagiaire à l'entreprise l'EPHEC en automatisation. Mon but est d'améliorer un essai de traction, je dois concevoir une acquisition de donnée. Pour cela, j'ai 3 capteurs à coder et l'envoyer dans LabVIEW.

## Matériel
[x] DHT22  
[x] Capteur de force  
[x] INA125PA  
[x] LCD  
[x] ESP32  
[ ] capteur de pression 

# Code & schéma
Mon esp32 est le point central de mon projet, c'est le microcôntroleur qui va envoyé mes données dans LabVIEW.  
Le DHT22 est un capteur d'environnemental qui détecte la température et l'humidité. Il va envoyé ses 2 données dans l'esp32 et l'afficher sur le LCD.  
Le capteur de force va envoyé une tension et celle étant trop petite, je dois l'amplifier avec le INA125PA puis l'envoyer dans l'esp32.  
Le capteur de pression, je ne l'ai pas encore.