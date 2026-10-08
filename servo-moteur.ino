// =============================================================
//  Noeud SERVOMOTEUR (ID 011) - Projet ZigBee ENSIM
//  XBee PRO 802.15.4, mode transparent, 9600 bauds
//
//  Trame (4 octets) :
//   [0] 0xAA                       synchro
//   [1] dest(3) | exp(3) | d9 d8   adressage + 2 bits de poids fort
//   [2] d7..d0                     donnee
//   [3] (o0 + o1 + o2) % 256       checksum
// =============================================================

#include <Servo.h>
#include <SoftwareSerial.h>

// ---------- Configuration ----------
#define DEBUG           1     // 1 = affiche tout sur le moniteur serie
#define REPONSE_ACTIVE  1     // 0 si le Hub n'attend pas de reponse

const uint8_t SYNC         = 0xAA;
const uint8_t MON_ID       = 0b011;  // Servomoteur
const uint8_t PIN_XBEE_RX  = 2;      // relie au DOUT du XBee
const uint8_t PIN_XBEE_TX  = 3;      // relie au DIN du XBee
const uint8_t PIN_SERVO    = 9;

// Servo a rotation continue
const unsigned long TEMPS_ROTATION = 3000;  // duree de rotation en ms, a ajuster sur la maquette
const uint8_t SERVO_OUVRIR = 180;    // echanger avec SERVO_FERMER si le sens est inverse
const uint8_t SERVO_FERMER = 0;
const uint8_t SERVO_ARRET  = 85;

// Commandes : A VALIDER avec le groupe Hub
const uint16_t CMD_FERMER = 0;
const uint16_t CMD_OUVRIR = 1;
const uint16_t CMD_ETAT   = 2;

SoftwareSerial xbee(PIN_XBEE_RX, PIN_XBEE_TX);
Servo servo;
bool storeOuvert = false;            // le store doit etre ferme au demarrage

// ---------- Outils trame ----------
uint8_t checksum(uint8_t a, uint8_t b, uint8_t c) {
  return (uint16_t(a) + b + c) % 256;
}

void afficherTrame(const char *titre, const uint8_t *t) {
#if DEBUG
  Serial.print(titre);
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(F(" 0x"));
    if (t[i] < 0x10) Serial.print('0');
    Serial.print(t[i], HEX);
  }
  Serial.println();
#endif
}

void envoyer(uint8_t dest, uint16_t data) {
  uint8_t t[4];
  t[0] = SYNC;
  t[1] = (dest << 5) | (MON_ID << 2) | ((data >> 8) & 0x03);
  t[2] = data & 0xFF;
  t[3] = checksum(t[0], t[1], t[2]);
  xbee.write(t, 4);
  afficherTrame("TX :", t);
}

// ---------- Actions ----------
void ouvrir() {
  if (storeOuvert) return;           // deja ouvert : on ne retourne pas
#if DEBUG
  Serial.println(F("Ouverture du store"));
#endif
  servo.attach(PIN_SERVO);
  servo.write(SERVO_OUVRIR);
  delay(TEMPS_ROTATION);
  servo.detach();                    // coupe le signal : arret complet
  storeOuvert = true;
}

void fermer() {
  if (!storeOuvert) return;          // deja ferme : on ne retourne pas
#if DEBUG
  Serial.println(F("Fermeture du store"));
#endif
  servo.attach(PIN_SERVO);
  servo.write(SERVO_FERMER);
  delay(TEMPS_ROTATION);
  servo.detach();                    // coupe le signal : arret complet
  storeOuvert = false;
}

void traiter(const uint8_t *t) {
  afficherTrame("RX :", t);

  if (checksum(t[0], t[1], t[2]) != t[3]) {
#if DEBUG
    Serial.println(F("  Checksum invalide, trame ignoree"));
#endif
    return;
  }

  uint8_t  dest = t[1] >> 5;
  uint8_t  exp  = (t[1] >> 2) & 0x07;
  uint16_t data = ((uint16_t)(t[1] & 0x03) << 8) | t[2];

#if DEBUG
  Serial.print(F("  dest=")); Serial.print(dest, BIN);
  Serial.print(F(" exp="));   Serial.print(exp, BIN);
  Serial.print(F(" data="));  Serial.println(data);
#endif

  if (dest != MON_ID) return;  // pas pour nous

  switch (data) {
    case CMD_OUVRIR: ouvrir(); break;
    case CMD_FERMER: fermer(); break;
    case CMD_ETAT:   break;
    default:
#if DEBUG
      Serial.println(F("  Commande inconnue"));
#endif
      return;
  }

#if REPONSE_ACTIVE
  // Reponse a l'expediteur : 1 = store ouvert, 0 = ferme
  envoyer(exp, storeOuvert ? 1 : 0);
#endif
}

// ---------- Programme principal ----------
void setup() {
  Serial.begin(9600);
  xbee.begin(9600);
  servo.attach(PIN_SERVO);
  servo.write(SERVO_ARRET);
#if DEBUG
  Serial.println(F("Servomoteur pret (ID 011)"));
#endif
}

void loop() {
  static uint8_t buf[4];
  static uint8_t idx = 0;
  static unsigned long dernierOctet = 0;

  // Trame incomplete depuis trop longtemps : on repart a zero
  if (idx > 0 && millis() - dernierOctet > 100) idx = 0;

  while (xbee.available()) {
    uint8_t b = xbee.read();
    dernierOctet = millis();

    if (idx == 0 && b != SYNC) continue;  // attente du 0xAA
    buf[idx++] = b;

    if (idx == 4) {
      idx = 0;
      traiter(buf);
    }
  }
}