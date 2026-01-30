#include <Wire.h> // a
#include <PCA9685.h>
#define DEBUG 1
PCA9685 pwmController;

// Variables de pilotage
byte vRecu = 50, dRecu = 50;
unsigned long chronoAR = 0;
int etapeAR = 0; 

void setup() {
    Serial.begin(115200); 
    Wire.begin(); // on allume le protocole I2C
    pwmController.resetDevices();
    pwmController.init();
    pwmController.setPWMFrequency(50);
    pwmController.setChannelPWM(1, 307); // On met le neutre tout de suite 
}

void loop() {
    // ÉTAPE 1 : LECTURE BINAIRE (On attend 4 octets)
    if (Serial.available() >= 4) { // l'arduino vérifie si il a bien reçu 4 octets.
        if (Serial.read() == 0xFF) { // on vérifie que le premier octet est bien 255
            byte v = Serial.read();  // v = 2eme octet (vitesse)
            byte d = Serial.read();  // d = 3eme octet (direction)
            byte cs = Serial.read(); /// cs = 4 eme octet (checksum)

            // ÉTAPE 2 : VALIDATION CHECKSUM
            if (cs == (byte)(v + d)) { 
                vRecu = v;
                dRecu = d;

                // DIRECTION (Canal 0)
                pwmController.setChannelPWM(0, map(dRecu, 0, 100, 388, 288)); // produit en croix

                // MOTEUR AVANT/STOP (Canal 1)
                if (vRecu >= 50) {
                    etapeAR = 0; // Stop la séquence arrière
                    if (vRecu == 50) {
                        pwmController.setChannelPWM(1, 307);
                    } else {
                        // Mapping Marche Avant
                        pwmController.setChannelPWM(1, map(vRecu, 51, 100, 325, 410));
                    }
                }
            }else{
              if(DEBUG)
                Serial.println("Mauvais CRC");
            }
        }
    }

    // ÉTAPE 3 : GESTION MARCHE ARRIERE (Non-bloquante)
    if (vRecu < 50) {
        gestionMarcheArriere();
    }
}

void gestionMarcheArriere() {
    unsigned long m = millis();
    int pwmCible = map(vRecu, 0, 49, 220, 285);

    switch (etapeAR) {
        case 0: pwmController.setChannelPWM(1, 205); chronoAR = m; etapeAR = 1; break; // Frein
        case 1: if (m - chronoAR >= 350) {
                      pwmController.setChannelPWM(1, 307); chronoAR = m; etapeAR = 2; 
                } 
                break; // Neutre
        case 2: if (m - chronoAR >= 350) { 
                    pwmController.setChannelPWM(1, 220);
                    chronoAR = m; etapeAR = 3; 
                }
                break; // Kickstart
        case 3: if (m - chronoAR >= 100) {
              pwmController.setChannelPWM(1, pwmCible);
              etapeAR = 4; 
              } 
              break; // Stabilise
        case 4: pwmController.setChannelPWM(1, pwmCible);
              break; // Roulage
    }
}
