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
                pwmController.setChannelPWM(0, map(dRecu, 0, 100, 388, 288)); // produit en croix on transforme mon 0-100 en ticks

                // MOTEUR AVANT/STOP (Canal 1)
                if (vRecu >= 50) {
                    etapeAR = 0; // Stop la séquence arrière
                    if (vRecu == 50) {
                        pwmController.setChannelPWM(1, 307);
                    } else {
                        // Mapping Marche Avant
                        pwmController.setChannelPWM(1, map(vRecu, 51, 100, 325, 410));// on commence à 325 pour que la voiture est assez de ticks pour démarrer, sinon le moteur peut siffler en dessous
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
        gestionMarcheArriere(); //  si la vitesses est < 50 on appel la fonction gestionMarcheArriere
    }
}

void gestionMarcheArriere() {
    unsigned long m = millis(); // chronomètre interne de l'arduino pour savoir combien de temps s'est écoulé
    int pwmCible = map(vRecu, 0, 49, 220, 285); // produit en croix, on transforme mon 0-49 en 220-285 ticks

    switch (etapeAR) {
        case 0: pwmController.setChannelPWM(1, 205); chronoAR = m; etapeAR = 1; break; // Frein // on envoie 205 et on lance le chrono (chrono AR = m) et on passe au case 1
        case 1: if (m - chronoAR >= 350) { 
                      pwmController.setChannelPWM(1, 307); chronoAR = m; etapeAR = 2; 
                } 
                break; // Si ça fait plus de 350ms qu'on freine alors on passe met le moteur au neutre et on passe au case 2
        case 2: if (m - chronoAR >= 350) { 
                    pwmController.setChannelPWM(1, 220);
                    chronoAR = m; etapeAR = 3; 
                }
                break; // Kickstart // on attend encore 350ms et puis on envoie le signal de recul (220) et on passe au case 3 
        case 3: if (m - chronoAR >= 100) {
              pwmController.setChannelPWM(1, pwmCible); 
              etapeAR = 4; // on attend 100ms et on envoie la vitesse cible désiré.
              } 
              break; // Stabilise
        case 4: pwmController.setChannelPWM(1, pwmCible);
              break; // Roulage
    }
}
