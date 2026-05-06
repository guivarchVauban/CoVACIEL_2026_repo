// Top départ XBee S1 - Coordinateur 802.15.4
// Shield Arduino : XBee TX -> pin 2, RX -> pin 3
// Moniteur série : 9600 baud, "Pas de fin de ligne"

#include <SoftwareSerial.h>

// SoftwareSerial(RX_arduino, TX_arduino)
// XBee TX -> pin 2 (on lit) | XBee RX -> pin 5 (on écrit)
SoftwareSerial xbee(2, 3);

void setup() {
  Serial.begin(9600);
  xbee.begin(9600);
  afficherMenu();
}

void loop() {
  if (Serial.available()) {
    char choix = Serial.read();

    switch (choix) {
      case '1':
        xbee.print("$GO;");
        Serial.println("[OK] Top depart envoye : $GO;");
        afficherMenu();
        break;

      case '2':
        xbee.print("STOP");
        Serial.println("[OK] Arret envoye : STOP");
        afficherMenu();
        break;

      case '\r':
      case '\n':
        break; // ignorer les retours chariot

      default:
        Serial.println("[!] Choix invalide");
        afficherMenu();
        break;
    }
  }
}

void afficherMenu() {
  Serial.println();
  Serial.println("===== MENU TOP DEPART =====");
  Serial.println("  1 -> Envoyer $GO;");
  Serial.println("  2 -> Envoyer STOP");
  Serial.println("===========================");
  Serial.print("Votre choix : ");
}
