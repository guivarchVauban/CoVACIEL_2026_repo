// ESC_Serial_Control.ino
// Contrôle ESC via Servo library, commandes série, ramp non-bloquant.
// Pin signal: 9 (OC1A, Timer1) => très fiable pour servo/ESC.

#include <Servo.h>

Servo esc;
const uint8_t ESC_PIN = 9;

// microsecond ranges for ESC (adaptable si ton ESC diffère)
const int ESC_MIN_US = 1500;        // 0% throttle
const int ESC_MAX_US = 2000;        // 100% throttle
const int ESC_ARM_US = ESC_MIN_US;  // position de sécurité pour armer

// --- ESC 1060 specific ---
const int ESC_NEUTRAL_US = 1500;
const int ESC_BRAKE_US = 1000;
const int ESC_REV_MIN_US = 1200;
const int ESC_REV_MAX_US = 1400;

const unsigned long REV_BRAKE_TIME = 300;    // ms
const unsigned long REV_NEUTRAL_TIME = 300;  // ms

// Serial
const unsigned long BAUD = 115200;

// ===== Direction servo =====
Servo steer;
const uint8_t STEER_PIN = 10;

// Ajuste ces valeurs à TA mécanique
const int STEER_CENTER_US = 1500;
const int STEER_LEFT_US = 1200;
const int STEER_RIGHT_US = 1800;

// Sécurité
int steerCurrentUs = STEER_CENTER_US;


// Ramping (non-bloquant)
int currentUs = ESC_MIN_US;
int targetUs = ESC_MIN_US;
unsigned long rampStartTime = 0;
unsigned long rampDuration = 0;  // en ms
int startUs = ESC_MIN_US;

// Etat
bool armed = false;
bool enabled = true;  // si false, bloc les sorties
unsigned long lastSerialEcho = 0;

bool demoMode = true;
unsigned long demoStepStart = 0;
int demoStep = 0;

void setup() {
  Serial.begin(BAUD);
  while (!Serial && millis() < 2000)
    ;  // attente légère si USB
  Serial.println(F("ESC Serial Control starting..."));

  esc.attach(ESC_PIN);
  // mettre le signal à min (sécurité) avant alimentation de l'ESC si possible
  esc.writeMicroseconds(ESC_ARM_US);
  currentUs = ESC_ARM_US;
  targetUs = ESC_ARM_US;
  startUs = ESC_ARM_US;


  steer.attach(STEER_PIN);
  steer.writeMicroseconds(STEER_CENTER_US);
  steerCurrentUs = STEER_CENTER_US;


  // Arm initial auto: on envoie la position min pendant 2000ms puis on considère armé.
  Serial.println(F("Auto-arm sequence: sending min throttle for 2000ms..."));
  delay(2000);
  armed = true;
  Serial.println(F("Armed (assume ESC armed if alimenté correctement)."));
  printHelp();
}

void loop() {
  // appliquer ramp si nécessaire
  updateRamp();
  updateDemo();

  // appliquer sortie si enabled
  if (enabled) esc.writeMicroseconds(currentUs);
  else esc.writeMicroseconds(ESC_MIN_US);  // forcer min si désactivé

  // lire série
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) handleCommand(line);
  }

  // périodique: afficher status toutes les 2s si inactif
  if (millis() - lastSerialEcho > 2000) {
    Serial.print(F("Status: "));
    Serial.print(armed ? "ARMED" : "DISARMED");
    Serial.print(F(" | currentUs="));
    Serial.print(currentUs);
    Serial.print(F(" | targetUs="));
    Serial.print(targetUs);
    Serial.print(F(" | enabled="));
    Serial.println(enabled ? "YES" : "NO");
    lastSerialEcho = millis();
  }
}

///////////////////// fonctions /////////////////////

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd == "ARM") {
    doArm();
    return;
  }
  if (cmd == "DISARM") {
    doDisarm();
    return;
  }
  if (cmd == "STOP") {
    doStop();
    return;
  }
  if (cmd == "STATUS") {
    printStatus();
    return;
  }
  if (cmd == "HELP" || cmd == "?") {
    printHelp();
    return;
  }
  // SET n  -> n en pourcentage 0..100
  if (cmd.startsWith("SET ")) {
    String val = cmd.substring(4);
    int perc = val.toInt();
    setPercentImmediate(constrain(perc, 0, 100));
    return;
  }
  // RAMP n ms -> monter vers n% en ms
  if (cmd.startsWith("RAMP ")) {
    // format: RAMP <percent> <duration_ms>
    int sp = -1;
    unsigned long dur = 0;
    // extraire deux tokens
    int spStart = 5;
    int spEnd = cmd.indexOf(' ', spStart);
    if (spEnd > 0) {
      sp = cmd.substring(spStart, spEnd).toInt();
      String durStr = cmd.substring(spEnd + 1);
      dur = (unsigned long)durStr.toInt();
      startRamp(constrain(sp, 0, 100), dur);
      return;
    } else {
      Serial.println(F("RAMP requires two args: RAMP <0..100> <ms>"));
      return;
    }
  }

  // REV n -> marche arrière en %
  if (cmd.startsWith("REV ")) {
    int perc = cmd.substring(4).toInt();
    doReverse(constrain(perc, 0, 100));
    return;
  }

  // STEER n -> direction -100..+100
  if (cmd.startsWith("STEER ")) {
    int perc = cmd.substring(6).toInt();
    setSteeringPercent(constrain(perc, -100, 100));
    return;
  }

  if (cmd == "DEMO") {
    startDemo();
    return;
  }
  if (cmd == "DEMO STOP") {
    stopDemo();
    return;
  }



  Serial.print(F("Commande inconnue: "));
  Serial.println(cmd);
  printHelp();
}

void doArm() {
  // convention: pour armer l'ESC tu dois fournir min throttle au moment de la mise sous tension.
  // Ici on envoie min pendant 5s puis on marque armed=true (vérifie ton ESC).
  Serial.println(F("Arming: sending min throttle 5000ms..."));
  esc.writeMicroseconds(ESC_MIN_US);
  delay(5000);
  armed = true;
  currentUs = ESC_MIN_US;
  targetUs = ESC_MIN_US;
  Serial.println(F("Armed."));
}

void doDisarm() {
  Serial.println(F("Disarming: setting min and disabling outputs."));
  enabled = false;
  esc.writeMicroseconds(ESC_MIN_US);
  armed = false;
  currentUs = ESC_MIN_US;
  targetUs = ESC_MIN_US;
  Serial.println(F("Disarmed."));
}

void doStop() {
  Serial.println(F("STOP: Ramp to 0% immediately."));
  setPercentImmediate(0);
}

void printStatus() {
  Serial.print(F("ARMED="));
  Serial.print(armed ? "YES" : "NO");
  Serial.print(F(" | enabled="));
  Serial.print(enabled ? "YES" : "NO");
  Serial.print(F(" | currentUs="));
  Serial.print(currentUs);
  Serial.print(F(" | targetUs="));
  Serial.println(targetUs);
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  ARM            -> armer l'ESC (min throttle envoyé)"));
  Serial.println(F("  DISARM         -> désarmer (bloque sortie)"));
  Serial.println(F("  SET <0..100>   -> définir throttle immédiat en pourcentage"));
  Serial.println(F("  RAMP <0..100> <ms> -> ramper vers la cible en <ms> ms"));
  Serial.println(F("  STOP           -> SET 0"));
  Serial.println(F("  STATUS         -> afficher état"));
  Serial.println(F("  HELP           -> ce message"));
  Serial.println(F("  REV <0..100>   -> marche arrière (procédure ESC 1060)"));
  Serial.println(F("  STEER -100..100 -> direction (gauche / droite)"));

  Serial.println(F("Ex: SET 30  or  RAMP 80 5000"));
}

//////////////// conversion % <-> microseconds //////////////////

int percentToUs(int percent) {
  // lineaire map 0..100 -> ESC_MIN_US..ESC_MAX_US
  long us = map(constrain(percent, 0, 100), 0, 100, ESC_MIN_US, ESC_MAX_US);
  return (int)us;
}

int usToPercent(int us) {
  long p = map(constrain(us, ESC_MIN_US, ESC_MAX_US), ESC_MIN_US, ESC_MAX_US, 0, 100);
  return (int)p;
}

void setPercentImmediate(int percent) {
  targetUs = percentToUs(percent);
  currentUs = targetUs;
  esc.writeMicroseconds(currentUs);
  Serial.print(F("SET immediate -> "));
  Serial.print(percent);
  Serial.print(F("% = "));
  Serial.print(currentUs);
  Serial.println(F(" us"));
  enabled = true;
}

void startRamp(int percent, unsigned long durationMs) {
  startUs = currentUs;
  targetUs = percentToUs(percent);
  rampStartTime = millis();
  rampDuration = durationMs;
  enabled = true;
  Serial.print(F("RAMP start -> target "));
  Serial.print(percent);
  Serial.print(F("% ("));
  Serial.print(targetUs);
  Serial.print(F("us) in "));
  Serial.print(durationMs);
  Serial.println(F(" ms"));
}

void updateRamp() {
  if (rampDuration == 0) return;
  unsigned long now = millis();
  unsigned long elapsed = now - rampStartTime;
  if (elapsed >= rampDuration) {
    currentUs = targetUs;
    rampDuration = 0;
    Serial.println(F("RAMP finished."));
  } else {
    // interpolation linéaire
    float t = (float)elapsed / (float)rampDuration;
    currentUs = (int)(startUs + t * (targetUs - startUs));
  }
}

void doReverse(int percent) {
  if (!armed) {
    Serial.println(F("ERROR: ESC not armed"));
    return;
  }

  percent = constrain(percent, 0, 100);

  // Calcul de l'impulsion reverse
  int revUs = map(percent, 0, 100, ESC_REV_MIN_US, ESC_REV_MAX_US);

  Serial.print(F("REVERSE sequence start -> "));
  Serial.print(percent);
  Serial.print(F("% ("));
  Serial.print(revUs);
  Serial.println(F(" us)"));

  enabled = false;  // on bloque les ramps pendant la séquence

  // 1️⃣ frein
  esc.writeMicroseconds(ESC_BRAKE_US);
  delay(REV_BRAKE_TIME);

  // 2️⃣ neutre
  esc.writeMicroseconds(ESC_NEUTRAL_US);
  delay(REV_NEUTRAL_TIME);

  // 3️⃣ reverse
  esc.writeMicroseconds(revUs);
  currentUs = revUs;
  targetUs = revUs;

  enabled = true;

  Serial.println(F("REVERSE engaged"));
}


void setSteeringPercent(int percent) {
  // percent : -100 (gauche) à +100 (droite)
  percent = constrain(percent, -100, 100);

  int us;
  if (percent < 0) {
    us = map(percent, -100, 0, STEER_LEFT_US, STEER_CENTER_US);
  } else {
    us = map(percent, 0, 100, STEER_CENTER_US, STEER_RIGHT_US);
  }

  steer.writeMicroseconds(us);
  steerCurrentUs = us;

  Serial.print(F("STEER -> "));
  Serial.print(percent);
  Serial.print(F("% ("));
  Serial.print(us);
  Serial.println(F(" us)"));
}

void steerCenter() {
  setSteeringPercent(0);
}

void startDemo() {
  Serial.println(F("DEMO mode START"));
  demoMode = true;
  demoStep = 0;
  demoStepStart = millis();
}

void stopDemo() {
  Serial.println(F("DEMO mode STOP"));
  demoMode = false;
  doStop();
  steerCenter();
}

void updateDemo() {
  if (!demoMode) return;

  unsigned long now = millis();

  switch (demoStep) {
    case 0:  // avant
      setSteeringPercent(0);
      setPercentImmediate(20);
      demoStepStart = now;
      demoStep++;
      break;

    case 1:  // rouler droit
      if (now - demoStepStart > 2000) {
        setSteeringPercent(20);  // droite
        demoStepStart = now;
        demoStep++;
      }
      break;

    case 2:  // virage droite
      if (now - demoStepStart > 1000) {
        setSteeringPercent(-20);  // gauche
        demoStepStart = now;
        demoStep++;
      }
      break;

    case 3:  // virage gauche
      if (now - demoStepStart > 1000) {
        steerCenter();
        doStop();
        demoStepStart = now;
        demoStep++;
      }
      break;

    case 4:  // arrêt
      if (now - demoStepStart > 2000) {
        doReverse(5);
        demoStepStart = now;
        demoStep++;
      }
      break;

    case 5:  // reverse
      if (now - demoStepStart > 2000) {
        doStop();
        steerCenter();
        demoStepStart = now;
        demoStep++;
      }
      break;

    default:
      demoStep = 0;  // boucle
      break;
  }
}
