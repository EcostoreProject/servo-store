// =============================================================
//  Noeud SERVOMOTEUR (NODE_SERVO_STORE = 5) - Projet ecostore ENSIM
//  XBee PRO 802.15.4, mode transparent, 9600 bauds
//
//  Trame (4 octets), geree par la librairie FrameProtocol :
//   [0] 0xAA                       debut
//   [1] dest(3) | src(3) | cmd(2)  adressage + type de commande
//   [2] value                      valeur sur 8 bits
//   [3] (o0 + o1 + o2) % 256       checksum
// =============================================================

#include <Servo.h>
#include <SoftwareSerial.h>
#include <frame_protocol.h>
#include <ecostore_nodes.h>

// ---------- Configuration ----------
#define DEBUG           1     // 1 = affiche tout sur le moniteur serie
#define REPONSE_ACTIVE  1     // 0 si le Hub n'attend pas de reponse

const uint8_t MON_ID       = NODE_SERVO_STORE;
const uint8_t PIN_XBEE_RX  = 2;      // relie au DOUT du XBee
const uint8_t PIN_XBEE_TX  = 3;      // relie au DIN du XBee
const uint8_t PIN_SERVO    = 9;

// Servo a rotation continue
const unsigned long TEMPS_ROTATION = 3000;  // duree de rotation en ms, a ajuster sur la maquette
const uint8_t SERVO_OUVRIR = 180;    // echanger avec SERVO_FERMER si le sens est inverse
const uint8_t SERVO_FERMER = 0;
const uint8_t SERVO_ARRET  = 85;

// Valeurs d'une trame WRITE : A VALIDER avec le groupe Hub
const uint8_t VAL_FERMER = 0;
const uint8_t VAL_OUVRIR = 1;

SoftwareSerial xbee(PIN_XBEE_RX, PIN_XBEE_TX);
Servo servo;
FrameParser_t parser;
bool storeOuvert = false;            // le store doit etre ferme au demarrage

// ---------- Outils trame ----------
void afficherMsg(const char *titre, const FrameMsg_t &msg) {
#if DEBUG
  Serial.print(titre);
  Serial.print(F(" dest="));  Serial.print(msg.dest_id);
  Serial.print(F(" src="));   Serial.print(msg.src_id);
  Serial.print(F(" cmd="));   Serial.print(msg.cmd);
  Serial.print(F(" value=")); Serial.println(msg.value);
#endif
}

void envoyer(uint8_t dest, FrameCmd_t cmd, uint8_t value) {
  FrameMsg_t msg;
  msg.dest_id = dest;
  msg.src_id  = MON_ID;
  msg.cmd     = cmd;
  msg.value   = value;

  uint8_t t[FRAME_TOTAL_SIZE];
  frame_pack(&msg, t);
  xbee.write(t, FRAME_TOTAL_SIZE);
  afficherMsg("TX :", msg);
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

void traiter(const FrameMsg_t &msg) {
  afficherMsg("RX :", msg);

  if (msg.dest_id != MON_ID) return;  // pas pour nous

  FrameCmd_t reponse = msg.cmd;

  if (msg.cmd == FRAME_CMD_WRITE) {
    if (msg.value == VAL_OUVRIR)      ouvrir();
    else if (msg.value == VAL_FERMER) fermer();
    else                              reponse = FRAME_CMD_ERROR;  // valeur inconnue
  } else if (msg.cmd != FRAME_CMD_READ) {
    return;                           // trame d'erreur recue : rien a faire
  }

#if REPONSE_ACTIVE
  // Reponse a l'expediteur : 1 = store ouvert, 0 = ferme
  envoyer(msg.src_id, reponse, storeOuvert ? 1 : 0);
#endif
}

// ---------- Programme principal ----------
void setup() {
  Serial.begin(9600);
  xbee.begin(XBEE_BAUD);
  frame_parser_init(&parser);
  servo.attach(PIN_SERVO);
  servo.write(SERVO_ARRET);
#if DEBUG
  Serial.println(F("Servomoteur pret (ID 5)"));
#endif
}

void loop() {
  FrameMsg_t msg;

  while (xbee.available()) {
    if (frame_parse_byte(&parser, xbee.read(), &msg)) {
      traiter(msg);
    }
  }
}