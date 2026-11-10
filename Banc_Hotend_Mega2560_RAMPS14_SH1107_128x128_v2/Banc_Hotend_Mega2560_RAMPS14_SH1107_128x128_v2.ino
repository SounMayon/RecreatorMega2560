/*
  BANC HOTEND + MOTEUR - MEGA 2560 / RAMPS 1.4
  OLED Grove 1.12" V2.1 SH1107G 128x128 (pilote valide par test materiel)

  Broches :
    E0 STEP D26, DIR D28, ENABLE D24 (DRV8825)
    Hotend D10, thermistance T0=A13
    Switch Fil NO D3 (INPUT_PULLUP : presse=LOW/In, relache=HIGH/Out)
    Joystick X=A5, Y=A9, bouton D40
    I2C SDA D20, SCL D21

  Navigation demandee :
    Accueil : Y+ = menu ; appui long bouton 1,5 s = gestion moteur.
    Gestion moteur : X+/X- = vitesse ; Y+ = FWD ; Y- = REV ;
      appui court = marche/arret ; appui long = quitter.
      Changement de sens : arret, changement DIR, redemarrage si actif.
    Menu : X+/X- = haut/bas ; Y+ ou bouton = selection ;
      Y- = retour ; bouton = valider une valeur.

  Switch Fil :
    Contact normalement ouvert entre D3 et GND.
    Presse = In ; relache = Out.
    Si surveillance Fil = ON et switch = Out, le moteur s'arrete.
    Il ne redemarre pas automatiquement : appui court en gestion moteur
    apres retour a In.

  IMPORTANT :
    MAX_RPM_MOTEUR est une limite logicielle de depart, pas une caracteristique
    garantie du moteur. Valider progressivement avec la charge reelle.
    Conserver une protection thermique materielle independante du logiciel.
    Le logiciel ne remplace ni fusible thermique, ni surveillance materielle.
*/

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <math.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stddef.h>
#include <EEPROM.h>

// ---------------- OLED : Grove V2.1 SH1107G, 128x128 ----------------
U8G2_SH1107_SEEED_128X128_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ---------------- Broches ----------------
const uint8_t PIN_STEP = 26;       // E0_STEP = PA4
const uint8_t PIN_DIR  = 28;       // E0_DIR
const uint8_t PIN_EN   = 24;       // E0_ENABLE, actif LOW
const uint8_t PIN_HEATER = 10;     // MOSFET hotend
const uint8_t PIN_THERM  = A13;    // T0
const uint8_t PIN_FIL    = 3;      // switch NO vers GND
const uint8_t PIN_JOY_X  = A5;
const uint8_t PIN_JOY_Y  = A9;
const uint8_t PIN_JOY_SW = 40;

// ---------------- Reglages a valider ----------------
const bool FIL_PRESSE_EST_LOW = true;
const bool FIL_ARRET_MOTEUR_SI_OUT = true;

const uint8_t DIR_FWD_LEVEL = LOW;
const uint8_t DIR_REV_LEVEL = HIGH;

const float PAS_PAR_TOUR = 200.0f * 32.0f;

const uint16_t MAX_RPM_MOTEUR = 120;
const uint8_t VITESSE_DEPART_PCT = 50;
const uint8_t PAS_VITESSE_PCT = 5;
const float RPM_DEMARRAGE = 8.0f;
const float ACCEL_RPM_PAR_SECONDE = 120.0f;

const float R_PULLUP = 4700.0f;
const float R_THERM_25 = 100000.0f;
const float BETA_THERM = 3950.0f;

struct PointThermistance { uint16_t adc; int16_t tempC; };
// Table nominale : 100 kOhm B3950, pull-up 4.7 kOhm, ADC 10 bits.
// Points tous les 10 C avec interpolation lineaire. Non calibree individuellement.
const PointThermistance TABLE_THERMISTANCE[] PROGMEM = {
  {1009,0},{1000,10},{986,20},{966,30},{940,40},
  {905,50},{860,60},{807,70},{747,80},{680,90},
  {611,100},{542,110},{475,120},{412,130},{355,140},
  {305,150},{261,160},{223,170},{191,180},{163,190},
  {140,200},{120,210},{104,220},{90,230},{78,240},
  {68,250},{60,260},{52,270},{46,280},{41,290},{36,300}
};
const uint8_t NB_POINTS_THERMISTANCE =
    sizeof(TABLE_THERMISTANCE) / sizeof(TABLE_THERMISTANCE[0]);

bool convertirADCEnTemperature(float adc, float &tempC) {
  const uint16_t adcFroid = pgm_read_word(&TABLE_THERMISTANCE[0].adc);
  const uint16_t adcChaud = pgm_read_word(&TABLE_THERMISTANCE[NB_POINTS_THERMISTANCE - 1].adc);
  if (!isfinite(adc) || adc > adcFroid || adc < adcChaud) return false;

  for (uint8_t n = 0; n < NB_POINTS_THERMISTANCE - 1; ++n) {
    const uint16_t adcHaut = pgm_read_word(&TABLE_THERMISTANCE[n].adc);
    const uint16_t adcBas = pgm_read_word(&TABLE_THERMISTANCE[n + 1].adc);
    if (adc <= adcHaut && adc >= adcBas) {
      const int16_t tempBasse = (int16_t)pgm_read_word(&TABLE_THERMISTANCE[n].tempC);
      const int16_t tempHaute = (int16_t)pgm_read_word(&TABLE_THERMISTANCE[n + 1].tempC);
      const float fraction = (adcHaut - adc) / (float)(adcHaut - adcBas);
      tempC = tempBasse + fraction * (tempHaute - tempBasse);
      return isfinite(tempC);
    }
  }
  return false;
}

const float TEMP_SECURITE_MAX = 290.0f;
const int TEMP_CONSIGNE_MIN = 50;
const int TEMP_CONSIGNE_MAX = 280;

float KP=8.8f, KI=0.33f, KD=58.3f;
const int EEPROM_PID_ADDR=0;
const uint32_t EEPROM_PID_MAGIC=0x50494432UL;
struct DonneesPID { uint32_t magic; uint8_t version; uint8_t reserved[3]; float kp,ki,kd; uint16_t crc; };
const float AUTOTUNE_HYST=2.0f, AUTOTUNE_PWM=180.0f;
const unsigned long AUTOTUNE_TIMEOUT=720000UL;
const unsigned long AUTOTUNE_TEST_PWM_MS=60000UL;
const float AUTOTUNE_MONTEE_MIN_C=2.0f;
unsigned long atTestDebut=0;
float atTestTemperature=0.0f;
enum EtatAutotune : uint8_t { AT_INACTIF, AT_EN_COURS, AT_OK, AT_ECHEC };
EtatAutotune autotune=AT_INACTIF;
unsigned long atDebut=0, atFront=0;
uint8_t atCycles=0;
float atPeriodes=0, atAmplitudes=0, atMin=999, atMax=-999;
float atAncienKP=22.0f, atAncienKI=1.08f, atAncienKD=114.0f;
bool atEepromVerifiee=false;
bool atChauffe=false, atPremier=true;
float atCible=0.0f, atNouveauKP=0.0f, atNouveauKI=0.0f, atNouveauKD=0.0f;
float atEepromKP=0.0f, atEepromKI=0.0f, atEepromKD=0.0f;
bool atEepromValide=false, atChoixOui=true, atSauvegardeRefusee=false;
float atLimiteHaute=0.0f;
enum EtatEcranAutotune : uint8_t { AT_UI_PREPARATION, AT_UI_RUN, AT_UI_SAUVEGARDE, AT_UI_TERMINE };
EtatEcranAutotune atEcran=AT_UI_PREPARATION;
const float BANDE_PID = 10.0f;
const unsigned long PERIODE_RUNAWAY_MS = 40000UL;
const float MONTEE_MIN_RUNAWAY = 2.0f;
const float RUNAWAY_ECART_C = 18.0f;
const unsigned long PERIODE_DERIVE_MS = 40000UL;
const unsigned long VEILLE_ECRAN_MS = 60000UL;

#define DEBUG_AU_DEMARRAGE 1
const unsigned long PERIODE_DEBUG_MS = 1000UL;
const unsigned long PERIODE_ECRAN_MS = 300UL;
const unsigned long APPUI_LONG_MS = 1500UL;
const unsigned long FIL_ANTI_REBOND_MS = 30UL;

enum ErreurChauffe : uint8_t { ERR_AUCUNE, ERR_SONDE, ERR_TEMP_MAX, ERR_RUNAWAY };
enum Direction : uint8_t { DIR_NONE, X_PLUS, X_MOINS, Y_PLUS, Y_MOINS };
enum EvenementBouton : uint8_t { BTN_AUCUN, BTN_COURT, BTN_LONG };
uint8_t erreurChauffe = ERR_AUCUNE;

bool chauffeOn = false;
int consigneTemp = 200;
float temperatureC = 0.0f;
bool sondeValide = false;
bool temperatureInitialisee = false;
float pwmChauffe = 0.0f;
float integralePID = 0.0f;
float deriveeFiltree = 0.0f;
float temperaturePrecedente = 0.0f;

bool moteurDemande = false;
bool moteurActif = false;
bool sensFwd = true;
uint8_t vitessePct = VITESSE_DEPART_PCT;
float rpmActuel = 0.0f;
bool arretFilMemorise = false;
bool changementSensEnAttente = false;
bool sensDemandeFwd = true;

bool filSurveillance = false;
bool filIn = false;
bool filInitialise = false;

enum ModeUI : uint8_t { UI_ACCUEIL, UI_MENU, UI_EDIT_TEMP, UI_GESTION_MOTEUR, UI_AUTOTUNE };
ModeUI modeUI = UI_ACCUEIL;
ModeUI modeRetourMoteur = UI_ACCUEIL;

enum ItemMenu : uint8_t {
  ITEM_CHAUFFE,
  ITEM_CONSIGNE,
  ITEM_MOTEUR,
  ITEM_FIL,
  ITEM_AUTOTUNE,
  NB_ITEMS
};
const char* const nomsMenu[NB_ITEMS] = {
  "Chauffe", "Temps C", "Moteur", "Capteur", "Autotune PID"
};
uint8_t selectionMenu = 0;

bool debugActif = DEBUG_AU_DEMARRAGE;
bool ecranSale = true;
bool ecranEnVeille = false;
unsigned long activiteAvantVeille = 0;
unsigned long derniereMajEcran = 0;
unsigned long derniereActivite = 0;
unsigned long dureeEcranMaxUs = 0;
unsigned long totalEcranUs = 0;
uint16_t nbMesuresEcran = 0;
const int EEPROM_CONFIG_ADDR = 32;
const uint32_t EEPROM_CONFIG_MAGIC = 0x43464731UL;
struct DonneesConfig { uint32_t magic; uint8_t version; int16_t consigne; uint8_t vitesse; uint8_t sensFwd; uint8_t filSurveillance; uint16_t crc; };
bool configSale = false;
unsigned long derniereModifConfig = 0;

ISR(TIMER4_COMPA_vect) {
  PORTA |= _BV(PA4);
  __builtin_avr_delay_cycles(48);
  PORTA &= ~_BV(PA4);
}

void timerMoteurDemarrer() {
  noInterrupts();
  TCCR4A = 0;
  TCCR4B = _BV(WGM42) | _BV(CS41);
  TCNT4 = 0;
  TIFR4 = _BV(OCF4A);
  TIMSK4 |= _BV(OCIE4A);
  interrupts();
}

void timerMoteurArreter() {
  noInterrupts();
  TIMSK4 &= ~_BV(OCIE4A);
  TCCR4B = 0;
  TIFR4 = _BV(OCF4A);
  interrupts();
}

void reglerFrequenceMoteur(float rpm) {
  if (rpm < 0.1f) rpm = 0.1f;
  const float frequencePas = rpm * PAS_PAR_TOUR / 60.0f;
  float top = 2000000.0f / frequencePas;
  top = constrain(top, 100.0f, 65535.0f);
  uint16_t valeur = (uint16_t)top - 1;

  noInterrupts();
  OCR4A = valeur;
  if (TCNT4 >= valeur) TCNT4 = 0;
  interrupts();
}

void arreterMoteurPhysiquement() {
  timerMoteurArreter();
  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_EN, HIGH);
  moteurActif = false;
  rpmActuel = 0.0f;
}

float rpmCible() {
  return (float)MAX_RPM_MOTEUR * (float)vitessePct / 100.0f;
}

bool filAutoriseMoteur() {
  return !(filSurveillance && FIL_ARRET_MOTEUR_SI_OUT && !filIn);
}
void demarrerMoteurPhysiquement() {
  if (!filAutoriseMoteur()) {
    moteurDemande = false;
    changementSensEnAttente = false;
    arretFilMemorise = true;
    if (debugActif) Serial.println(F("[FIL] Demarrage refuse : Fil=Out."));
    ecranSale = true;
    return;
  }

  digitalWrite(PIN_DIR, sensFwd ? DIR_FWD_LEVEL : DIR_REV_LEVEL);
  digitalWrite(PIN_EN, LOW);
  delayMicroseconds(5);

  float cible = rpmCible();
  rpmActuel = min((float)RPM_DEMARRAGE, cible);
  if (rpmActuel < 0.1f) rpmActuel = cible;
  reglerFrequenceMoteur(rpmActuel);
  timerMoteurDemarrer();
  moteurActif = true;
  if (debugActif) {
    Serial.print(F("[MOTEUR] Demarrage "));
    Serial.print(sensFwd ? F("FWD") : F("REV"));
    Serial.print(F(" a "));
    Serial.print(vitessePct);
    Serial.println(F("%"));
  }
  ecranSale = true;
}

void demanderArretMoteur() {
  moteurDemande = false;
  ecranSale = true;
}

void appliquerSens(bool nouveauSensFwd) {
  if (nouveauSensFwd == sensFwd || changementSensEnAttente) return;
  if (!moteurActif) {
    sensFwd = nouveauSensFwd;
    digitalWrite(PIN_DIR, sensFwd ? DIR_FWD_LEVEL : DIR_REV_LEVEL);
    marquerConfigSale();
  } else {
    sensDemandeFwd = nouveauSensFwd;
    changementSensEnAttente = true;
  }
  ecranSale = true;
}

void gererRampeMoteur() {
  static unsigned long precedent = 0;
  unsigned long maintenant = millis();
  if (maintenant - precedent < 10) return;
  float dt = (precedent == 0) ? 0.01f : (maintenant - precedent) / 1000.0f;
  precedent = maintenant;

  if (moteurDemande && !moteurActif) {
    demarrerMoteurPhysiquement();
    return;
  }
  if (!moteurActif) return;

  if (!filAutoriseMoteur()) {
    moteurDemande = false;
    changementSensEnAttente = false;
    arretFilMemorise = true;
    arreterMoteurPhysiquement();
    if (debugActif) Serial.println(F("[FIL] Fil=Out : arret moteur."));
    ecranSale = true;
    return;
  }

  float cible = (moteurDemande && !changementSensEnAttente) ? rpmCible() : 0.0f;
  float variation = ACCEL_RPM_PAR_SECONDE * dt;

  if (rpmActuel < cible) rpmActuel = min(rpmActuel + variation, cible);
  else if (rpmActuel > cible) rpmActuel = max(rpmActuel - variation, cible);

  if ((!moteurDemande || changementSensEnAttente) && rpmActuel <= RPM_DEMARRAGE) {
    arreterMoteurPhysiquement();
    if (changementSensEnAttente) {
      sensFwd = sensDemandeFwd;
      digitalWrite(PIN_DIR, sensFwd ? DIR_FWD_LEVEL : DIR_REV_LEVEL);
      delayMicroseconds(10);
      changementSensEnAttente = false;
      marquerConfigSale();
      if (moteurDemande && filAutoriseMoteur()) demarrerMoteurPhysiquement();
      else if (!filAutoriseMoteur()) { moteurDemande = false; arretFilMemorise = true; }
    }
    return;
  }
  reglerFrequenceMoteur(rpmActuel);
}

uint16_t crcPID(const uint8_t* p, size_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= *p++;
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
  }
  return crc;
}

bool pidValide(float p, float i, float d) {
  return isfinite(p) && isfinite(i) && isfinite(d) &&
         p >= 1 && p <= 200 && i >= 0.01f && i <= 20 &&
         d >= 0 && d <= 1000;
}

void chargerPID() {
  DonneesPID v;
  EEPROM.get(EEPROM_PID_ADDR, v);

  bool valide = v.magic == EEPROM_PID_MAGIC && v.version == 1 &&
      v.crc == crcPID((const uint8_t*)&v, offsetof(DonneesPID, crc)) &&
      pidValide(v.kp, v.ki, v.kd);

  if (valide) {
    KP = v.kp;
    KI = v.ki;
    KD = v.kd;
  }
}

bool lirePIDDepuisEEPROM(float &p, float &i, float &d) {
  DonneesPID v;
  EEPROM.get(EEPROM_PID_ADDR, v);

  bool valide = v.magic == EEPROM_PID_MAGIC && v.version == 1 &&
      v.crc == crcPID((const uint8_t*)&v, offsetof(DonneesPID, crc)) &&
      pidValide(v.kp, v.ki, v.kd);

  if (valide) {
    p = v.kp;
    i = v.ki;
    d = v.kd;
  }
  return valide;
}

bool sauverPIDValeurs(float p, float i, float d) {
  if (!pidValide(p, i, d)) return false;

  DonneesPID v = {EEPROM_PID_MAGIC, 1, {0, 0, 0}, p, i, d, 0};
  v.crc = crcPID((const uint8_t*)&v, offsetof(DonneesPID, crc));
  EEPROM.put(EEPROM_PID_ADDR, v);

  DonneesPID controle;
  EEPROM.get(EEPROM_PID_ADDR, controle);
  return controle.magic == v.magic && controle.version == v.version &&
      controle.crc == crcPID((const uint8_t*)&controle, offsetof(DonneesPID, crc)) &&
      pidValide(controle.kp, controle.ki, controle.kd) &&
      controle.kp == p && controle.ki == i && controle.kd == d;
}

void marquerConfigSale() {
  configSale = true;
  derniereModifConfig = millis();
}

void chargerConfig() {
  DonneesConfig c;
  EEPROM.get(EEPROM_CONFIG_ADDR, c);

  bool valide = c.magic == EEPROM_CONFIG_MAGIC && c.version == 1 &&
      c.crc == crcPID((const uint8_t*)&c, offsetof(DonneesConfig, crc)) &&
      c.consigne >= TEMP_CONSIGNE_MIN && c.consigne <= TEMP_CONSIGNE_MAX &&
      c.vitesse >= 5 && c.vitesse <= 100 &&
      c.sensFwd <= 1 && c.filSurveillance <= 1;

  if (valide) {
    consigneTemp = c.consigne;
    vitessePct = c.vitesse;
    sensFwd = c.sensFwd != 0;
    filSurveillance = c.filSurveillance != 0;
  }
}

void sauverConfigSiNecessaire() {
  if (!configSale || millis() - derniereModifConfig < 1500UL) return;

  DonneesConfig c = {
    EEPROM_CONFIG_MAGIC, 1, (int16_t)consigneTemp, vitessePct,
    (uint8_t)(sensFwd ? 1 : 0),
    (uint8_t)(filSurveillance ? 1 : 0), 0
  };
  c.crc = crcPID((const uint8_t*)&c, offsetof(DonneesConfig, crc));
  EEPROM.put(EEPROM_CONFIG_ADDR, c);

  DonneesConfig controle;
  EEPROM.get(EEPROM_CONFIG_ADDR, controle);
  configSale = !(controle.magic == c.magic &&
      controle.version == c.version &&
      controle.crc == crcPID((const uint8_t*)&controle, offsetof(DonneesConfig, crc)) &&
      controle.consigne == c.consigne &&
      controle.vitesse == c.vitesse &&
      controle.sensFwd == c.sensFwd &&
      controle.filSurveillance == c.filSurveillance);

  if (debugActif) {
    Serial.println(configSale
      ? F("[EEPROM] Config: verification ECHEC")
      : F("[EEPROM] Config sauvegardee/verifiee"));
  }
}

void finirAutotune(bool ok) {
  analogWrite(PIN_HEATER, 0);
  pwmChauffe = 0;
  chauffeOn = false;
  atChauffe = false;
  atEepromVerifiee = false;

  if (ok) {
    autotune = AT_OK;
    atEcran = AT_UI_SAUVEGARDE;
    atChoixOui = true;
    atSauvegardeRefusee = false;
  } else {
    KP = atAncienKP;
    KI = atAncienKI;
    KD = atAncienKD;
    autotune = AT_ECHEC;
    atEcran = AT_UI_TERMINE;
  }
  ecranSale = true;
}

bool preparerAutotune() {
  if (!sondeValide || !temperatureInitialisee ||
      erreurChauffe != ERR_AUCUNE ||
      consigneTemp < TEMP_CONSIGNE_MIN ||
      consigneTemp > TEMP_CONSIGNE_MAX ||
      temperatureC >= consigneTemp + 15.0f ||
      temperatureC >= TEMP_SECURITE_MAX - 20.0f ||
      moteurActif || moteurDemande) {
    return false;
  }

  atAncienKP = KP;
  atAncienKI = KI;
  atAncienKD = KD;
  atEepromValide = lirePIDDepuisEEPROM(atEepromKP, atEepromKI, atEepromKD);
  atCible = (float)consigneTemp;
  atNouveauKP = atAncienKP;
  atNouveauKI = atAncienKI;
  atNouveauKD = atAncienKD;
  atEepromVerifiee = false;
  atSauvegardeRefusee = false;
  atChoixOui = true;

  atLimiteHaute = min(atCible + 20.0f, TEMP_SECURITE_MAX - 10.0f);
  if (atLimiteHaute <= atCible + AUTOTUNE_HYST) {
    if (debugActif) {
      Serial.println(F("[AUTOTUNE] Refuse: marge thermique insuffisante au-dessus de la cible."));
    }
    return false;
  }

  autotune = AT_INACTIF;
  atEcran = AT_UI_PREPARATION;
  modeUI = UI_AUTOTUNE;

  if (debugActif) {
    Serial.print(F("[AUTOTUNE] PREPARATION cible="));
    Serial.print(atCible, 1);
    Serial.print(F(" limite="));
    Serial.print(atLimiteHaute, 1);
    Serial.print(F(" EEPROM="));
    Serial.println(atEepromValide ? F("VALIDE") : F("INVALIDE/VIDE"));
    if (atEepromValide) {
      Serial.print(F("[AUTOTUNE] PID EEPROM Kp="));
      Serial.print(atEepromKP, 3);
      Serial.print(F(" Ki="));
      Serial.print(atEepromKI, 3);
      Serial.print(F(" Kd="));
      Serial.println(atEepromKD, 3);
    }
  }

  ecranSale = true;
  return true;
}

bool lancerAutotune() {
  if (!sondeValide || !temperatureInitialisee ||
      erreurChauffe != ERR_AUCUNE ||
      temperatureC >= atCible + 15.0f ||
      temperatureC >= atLimiteHaute - 5.0f ||
      moteurActif || moteurDemande) {
    return false;
  }

  chauffeOn = false;
  couperChauffage();
  integralePID = 0;
  deriveeFiltree = 0;

  atDebut = millis();
  atTestDebut = atDebut;
  atTestTemperature = temperatureC;
  atFront = 0;
  atCycles = 0;
  atPeriodes = 0;
  atAmplitudes = 0;
  atMin = 999;
  atMax = -999;
  atPremier = true;
  atChauffe = true;

  autotune = AT_EN_COURS;
  atEcran = AT_UI_RUN;
  analogWrite(PIN_HEATER, (int)AUTOTUNE_PWM);

  if (debugActif) {
    Serial.print(F("[AUTOTUNE] DEMARRE cible="));
    Serial.print(atCible, 1);
    Serial.print(F(" plafond="));
    Serial.println(atLimiteHaute, 1);
  }

  modeUI = UI_AUTOTUNE;
  ecranSale = true;
  return true;
}

void gererAutotune(float t, unsigned long now) {
  if (autotune != AT_EN_COURS) return;

  if (!sondeValide || !temperatureInitialisee ||
      erreurChauffe != ERR_AUCUNE ||
      t >= atLimiteHaute ||
      now - atDebut > AUTOTUNE_TIMEOUT) {
    if (debugActif) {
      Serial.println(F("[AUTOTUNE] ABANDON: securite, sonde, plafond ou delai."));
    }
    finirAutotune(false);
    return;
  }

  // Verifier que la puissance fixe de 180/255 produit une montee mesurable.
  if (atChauffe && now - atTestDebut >= AUTOTUNE_TEST_PWM_MS) {
    if (t - atTestTemperature < AUTOTUNE_MONTEE_MIN_C) {
      if (debugActif) {
        Serial.println(F("[AUTOTUNE] PWM 180 insuffisant: montee < 2 C en 60 s."));
      }
      finirAutotune(false);
      return;
    }
    atTestDebut = now;
    atTestTemperature = t;
  }

  if (t < atMin) atMin = t;
  if (t > atMax) atMax = t;

  bool next = atChauffe;
  if (atChauffe && t >= atCible + AUTOTUNE_HYST) next = false;
  if (!atChauffe && t <= atCible - AUTOTUNE_HYST) next = true;

  if (next != atChauffe) {
    if (!next) {
      if (!atPremier) {
        float periode = (now - atFront) / 1000.0f;
        float amplitude = (atMax - atMin) * 0.5f;
        if (periode >= 5 && periode <= 180 &&
            amplitude >= 0.5f && amplitude <= 20) {
          atPeriodes += periode;
          atAmplitudes += amplitude;
          atCycles++;
        }
      }
      atFront = now;
      atPremier = false;
      atMin = t;
      atMax = t;
      ecranSale = true;
    } else {
      // Chaque phase ON dispose de sa propre fenetre de test de puissance.
      atTestDebut = now;
      atTestTemperature = t;
    }

    atChauffe = next;
    analogWrite(PIN_HEATER, next ? (int)AUTOTUNE_PWM : 0);
  }

  if (atCycles >= 5) {
    float tu = atPeriodes / atCycles;
    float amplitude = atAmplitudes / atCycles;
    if (tu <= 0.0f || amplitude <= 0.0f) {
      finirAutotune(false);
      return;
    }

    float ku = (4.0f * (AUTOTUNE_PWM * 0.5f)) /
               (3.14159265f * amplitude);
    float p = 0.60f * ku;
    float i = 1.20f * ku / tu;
    float d = 0.075f * ku * tu;

    if (pidValide(p, i, d)) {
      atNouveauKP = p;
      atNouveauKI = i;
      atNouveauKD = d;
      if (debugActif) {
        Serial.print(F("[AUTOTUNE] CALCULE Kp="));
        Serial.print(p, 3);
        Serial.print(F(" Ki="));
        Serial.print(i, 3);
        Serial.print(F(" Kd="));
        Serial.println(d, 3);
      }
      finirAutotune(true);
    } else {
      finirAutotune(false);
    }
  }
}

bool lireTemperature(float &tempC) {
  uint32_t somme = 0;
  for (uint8_t i = 0; i < 16; ++i) {
    somme += analogRead(PIN_THERM);
  }
  const float adc = somme / 16.0f;
  return convertirADCEnTemperature(adc, tempC);
}

void couperChauffage() {
  pwmChauffe = 0.0f;
  analogWrite(PIN_HEATER, 0);
}

void declencherErreurChauffe(uint8_t code) {
  chauffeOn = false;
  erreurChauffe = code;
  integralePID = 0.0f;
  deriveeFiltree = 0.0f;
  couperChauffage();
  ecranSale = true;

  if (debugActif) {
    Serial.print(F("[CHAUFFAGE] DEFAUT code="));
    Serial.println(code);
  }}

void gererChauffage() {
  static unsigned long precedent = 0;
  static unsigned long debutControleRunaway = 0;
  static float temperatureReference = 0.0f;
  static bool regulationArmee = false;
  static unsigned long debutDerive = 0;

  unsigned long maintenant = millis();
  if (maintenant - precedent < 200) return;
  float dt = (precedent == 0) ? 0.2f : (maintenant - precedent) / 1000.0f;
  precedent = maintenant;

  float mesure;
  sondeValide = lireTemperature(mesure);
  if (!sondeValide) {
    if (autotune == AT_EN_COURS) finirAutotune(false);
    temperatureInitialisee = false;
    couperChauffage();
    if (erreurChauffe == ERR_AUCUNE) declencherErreurChauffe(ERR_SONDE);
    chauffeOn = false;
    return;
  }

  if (!temperatureInitialisee) {
    temperatureC = mesure;
    temperatureInitialisee = true;
  } else {
    temperatureC = 0.7f * temperatureC + 0.3f * mesure;
  }

  if (mesure >= TEMP_SECURITE_MAX || temperatureC >= TEMP_SECURITE_MAX) {
    if (autotune == AT_EN_COURS) finirAutotune(false);
    if (erreurChauffe != ERR_TEMP_MAX) declencherErreurChauffe(ERR_TEMP_MAX);
    chauffeOn = false;
    couperChauffage();
    return;
  }
  if (erreurChauffe != ERR_AUCUNE) {
    chauffeOn = false;
    couperChauffage();
    return;
  }

  if(autotune==AT_EN_COURS) { gererAutotune(mesure,maintenant); return; }
  if (!chauffeOn) {
    regulationArmee = false;
    debutDerive = 0;
    couperChauffage();
    integralePID = 0.0f;
    deriveeFiltree = 0.0f;
    temperaturePrecedente = temperatureC;
    temperatureReference = temperatureC;
    debutControleRunaway = maintenant;
    return;
  }

  float erreur = consigneTemp - temperatureC;
  if (temperatureC >= consigneTemp - BANDE_PID) regulationArmee = true;
  if (regulationArmee && temperatureC < consigneTemp - RUNAWAY_ECART_C) {
    if (debutDerive == 0) debutDerive = maintenant;
    else if (maintenant - debutDerive >= PERIODE_DERIVE_MS) {
      declencherErreurChauffe(ERR_RUNAWAY);
      if (debugActif) Serial.println(F("[CHAUFFAGE] RUNAWAY: ecart durable sous consigne."));
      return;
    }
  } else debutDerive = 0;

  if (erreur > BANDE_PID) {
    pwmChauffe = 255.0f;
    integralePID = 0.0f;
  } else if (erreur < -BANDE_PID) {
    pwmChauffe = 0.0f;
    integralePID = 0.0f;
  } else {
    integralePID += KI * erreur * dt;
    integralePID = constrain(integralePID, 0.0f, 255.0f);

    float derivee = (temperatureC - temperaturePrecedente) / dt;
    deriveeFiltree = 0.8f * deriveeFiltree + 0.2f * derivee;
    pwmChauffe = constrain(KP * erreur + integralePID - KD * deriveeFiltree,
                           0.0f, 255.0f);
  }
  temperaturePrecedente = temperatureC;

  if (pwmChauffe >= 254.0f && temperatureC < consigneTemp - BANDE_PID) {
    if (maintenant - debutControleRunaway >= PERIODE_RUNAWAY_MS) {
      if (temperatureC - temperatureReference < MONTEE_MIN_RUNAWAY) {
        declencherErreurChauffe(ERR_RUNAWAY);
        return;
      }
      temperatureReference = temperatureC;
      debutControleRunaway = maintenant;
    }
  } else {
    temperatureReference = temperatureC;
    debutControleRunaway = maintenant;
  }

  analogWrite(PIN_HEATER, (int)pwmChauffe);
}

void lireFil() {
  static bool niveauBrutPrecedent = HIGH;
  static bool niveauStable = HIGH;
  static unsigned long changementBrut = 0;
  bool niveau = digitalRead(PIN_FIL);
  unsigned long maintenant = millis();
  if (!filInitialise) {
    niveauBrutPrecedent = niveau;
    niveauStable = niveau;
    changementBrut = maintenant;
    filInitialise = true;
  }
  if (niveau != niveauBrutPrecedent) {
    niveauBrutPrecedent = niveau;
    changementBrut = maintenant;
  }
  if (maintenant - changementBrut >= FIL_ANTI_REBOND_MS && niveauStable != niveauBrutPrecedent)
    niveauStable = niveauBrutPrecedent;
  filIn = FIL_PRESSE_EST_LOW ? (niveauStable == LOW) : (niveauStable == HIGH);
}

uint8_t lireDirectionBrute() {
  int x = analogRead(PIN_JOY_X) - 512;
  int y = analogRead(PIN_JOY_Y) - 512;
  const int seuil = 220;

  if (abs(x) < seuil && abs(y) < seuil) return DIR_NONE;
  if (abs(x) >= abs(y)) return (x > 0) ? X_PLUS : X_MOINS;
  return (y > 0) ? Y_PLUS : Y_MOINS;
}

uint8_t lireDirectionEvenement() {
  static uint8_t precedente = DIR_NONE;
  static unsigned long prochaineRepetition = 0;
  uint8_t actuelle = lireDirectionBrute();
  unsigned long maintenant = millis();

  if (actuelle == DIR_NONE) {
    precedente = DIR_NONE;
    return DIR_NONE;
  }
  if (actuelle != precedente) {
    precedente = actuelle;
    prochaineRepetition = maintenant + 350;
    return actuelle;
  }
  if ((actuelle == X_PLUS || actuelle == X_MOINS) &&
      (long)(maintenant - prochaineRepetition) >= 0) {
    prochaineRepetition = maintenant + 130;
    return actuelle;
  }
  return DIR_NONE;
}

uint8_t lireEvenementBouton() {
  static bool lecturePrecedente = HIGH;
  static bool etatStable = HIGH;
  static unsigned long changement = 0;
  static unsigned long debutAppui = 0;
  static bool longEnvoye = false;

  bool lecture = digitalRead(PIN_JOY_SW);
  unsigned long maintenant = millis();

  if (lecture != lecturePrecedente) {
    lecturePrecedente = lecture;
    changement = maintenant;
  }

  if (maintenant - changement > 30 && lecture != etatStable) {
    etatStable = lecture;
    if (etatStable == LOW) {
      debutAppui = maintenant;
      longEnvoye = false;
    } else {
      if (!longEnvoye) return BTN_COURT;
    }
  }

  if (etatStable == LOW && !longEnvoye &&
      maintenant - debutAppui >= APPUI_LONG_MS) {
    longEnvoye = true;
    return BTN_LONG;
  }
  return BTN_AUCUN;
}

void activerChauffage() {
  if (chauffeOn) {
    chauffeOn = false;
    couperChauffage();
    if (debugActif) Serial.println(F("[CHAUFFAGE] Arret utilisateur."));
  } else if (!sondeValide || !temperatureInitialisee) {
    erreurChauffe = ERR_SONDE;
    chauffeOn = false;
    couperChauffage();
  } else if (temperatureC >= TEMP_SECURITE_MAX) {
    erreurChauffe = ERR_TEMP_MAX;
    chauffeOn = false;
    couperChauffage();
  } else {
    erreurChauffe = ERR_AUCUNE;
    chauffeOn = true;
    if (debugActif) Serial.println(F("[CHAUFFAGE] Active."));
  }
  ecranSale = true;
}

void selectionnerMenu() {
  switch(selectionMenu) {
    case ITEM_CHAUFFE: activerChauffage(); break;
    case ITEM_CONSIGNE: modeUI=UI_EDIT_TEMP; break;
    case ITEM_MOTEUR: modeRetourMoteur=UI_MENU; modeUI=UI_GESTION_MOTEUR; break;
    case ITEM_FIL:
      filSurveillance=!filSurveillance;
      marquerConfigSale();
      if(filSurveillance&&!filIn&&moteurActif){moteurDemande=false;changementSensEnAttente=false;arretFilMemorise=true;arreterMoteurPhysiquement();}
      if(debugActif){Serial.print(F("[FIL] Surveillance "));Serial.println(filSurveillance?F("ON"):F("OFF"));}
      break;
    case ITEM_AUTOTUNE:
      if(!preparerAutotune()&&debugActif)Serial.println(F("[AUTOTUNE] Refuse: verifier sonde, temperature, consigne et moteur arrete."));
      break;
  }
  ecranSale=true;
}

void ajusterVitesse(int variation) {
  int nouvelle = (int)vitessePct + variation;
  nouvelle = constrain(nouvelle, 5, 100);
  vitessePct = (uint8_t)nouvelle;
  marquerConfigSale();
  if (debugActif) {
    Serial.print(F("[MOTEUR] Vitesse demandee "));
    Serial.print(vitessePct);
    Serial.println(F("%"));
  }
  ecranSale = true;
}

void gererInterface() {
  uint8_t d=lireDirectionEvenement(), b=lireEvenementBouton();
  if(d!=DIR_NONE||b!=BTN_AUCUN) derniereActivite=millis();
  if(modeUI==UI_AUTOTUNE) {
    if(atEcran==AT_UI_PREPARATION) {
      if(d==X_PLUS||d==X_MOINS){atChoixOui=!atChoixOui;ecranSale=true;}
      if(d==Y_MOINS||b==BTN_LONG){modeUI=UI_MENU;ecranSale=true;return;}
      if(d==Y_PLUS||b==BTN_COURT){if(atChoixOui){if(!lancerAutotune()){autotune=AT_ECHEC;atEcran=AT_UI_TERMINE;}}else modeUI=UI_MENU;ecranSale=true;}
    } else if(atEcran==AT_UI_RUN) {
      if(b==BTN_COURT||b==BTN_LONG||d==Y_MOINS){finirAutotune(false);if(debugActif)Serial.println(F("[AUTOTUNE] Annule par utilisateur."));modeUI=UI_MENU;ecranSale=true;}
    } else if(atEcran==AT_UI_SAUVEGARDE) {
      if(d==X_PLUS||d==X_MOINS){atChoixOui=!atChoixOui;ecranSale=true;}
      if(d==Y_MOINS||b==BTN_LONG)atChoixOui=false;
      if(d==Y_PLUS||b==BTN_COURT||d==Y_MOINS||b==BTN_LONG){
        if(atChoixOui){atEepromVerifiee=sauverPIDValeurs(atNouveauKP,atNouveauKI,atNouveauKD);if(atEepromVerifiee){KP=atNouveauKP;KI=atNouveauKI;KD=atNouveauKD;}else{KP=atAncienKP;KI=atAncienKI;KD=atAncienKD;}if(debugActif)Serial.println(atEepromVerifiee?F("[AUTOTUNE] EEPROM sauvegardee et relue OK."):F("[AUTOTUNE] ERREUR EEPROM: anciens PID conserves en RAM."));}
        else{KP=atAncienKP;KI=atAncienKI;KD=atAncienKD;atSauvegardeRefusee=true;if(debugActif)Serial.println(F("[AUTOTUNE] Sauvegarde refusee: anciens PID conserves."));}
        atEcran=AT_UI_TERMINE;ecranSale=true;
      }
    } else if(b==BTN_COURT||b==BTN_LONG||d==Y_MOINS||d==Y_PLUS){modeUI=UI_MENU;ecranSale=true;}
    return;
  }
  switch(modeUI){
    case UI_ACCUEIL:
      if(d==Y_PLUS){modeUI=UI_MENU;selectionMenu=0;ecranSale=true;}
      if(b==BTN_LONG){modeRetourMoteur=UI_ACCUEIL;modeUI=UI_GESTION_MOTEUR;ecranSale=true;}
      break;
    case UI_GESTION_MOTEUR:
      if(d==X_PLUS)ajusterVitesse(PAS_VITESSE_PCT);else if(d==X_MOINS)ajusterVitesse(-PAS_VITESSE_PCT);else if(d==Y_PLUS)appliquerSens(true);else if(d==Y_MOINS)appliquerSens(false);
      if(b==BTN_COURT){if(moteurDemande)demanderArretMoteur();else if(filAutoriseMoteur()){moteurDemande=true;arretFilMemorise=false;}else if(debugActif)Serial.println(F("[MOTEUR] Bloque : Fil=Out."));ecranSale=true;}
      if(b==BTN_LONG){modeUI=modeRetourMoteur;ecranSale=true;}
      break;
    case UI_MENU:
      if(d==X_PLUS){selectionMenu=(selectionMenu+NB_ITEMS-1)%NB_ITEMS;ecranSale=true;}else if(d==X_MOINS){selectionMenu=(selectionMenu+1)%NB_ITEMS;ecranSale=true;}
      if(d==Y_PLUS||b==BTN_COURT)selectionnerMenu();
      if(d==Y_MOINS){modeUI=UI_ACCUEIL;ecranSale=true;}
      break;
    case UI_EDIT_TEMP:
      if(d==X_PLUS){consigneTemp=min(consigneTemp+1,TEMP_CONSIGNE_MAX);marquerConfigSale();ecranSale=true;}else if(d==X_MOINS){consigneTemp=max(consigneTemp-1,TEMP_CONSIGNE_MIN);marquerConfigSale();ecranSale=true;}
      if(b==BTN_COURT||d==Y_MOINS){modeUI=UI_MENU;ecranSale=true;}
      break;
    default:break;
  }
}

// ---------------- Dessin OLED ----------------

void dessinerAccueil() {
  char ligne[24];

  // ==========================================
  // LIGNE 1 : Tc (13 caractères max)
  // ==========================================
  u8g2.setFont(u8g2_font_9x15_tf); 
  if (erreurChauffe != ERR_AUCUNE) {
    const char* defaut = (erreurChauffe == ERR_SONDE) ? "ERR SONDE" :
                         (erreurChauffe == ERR_TEMP_MAX) ? "ERR TEMP MAX" : "ERR RUNAWAY";
    snprintf(ligne, sizeof(ligne), "%s", defaut);
  } else if (chauffeOn) {
    snprintf(ligne, sizeof(ligne), "Tc: %3d    ON", consigneTemp);
    u8g2.drawVLine(88, 0, 15); 
  } else {
    snprintf(ligne, sizeof(ligne), "Tc: ---    OFF");
    u8g2.drawVLine(88, 0, 15); 
  }
  u8g2.drawUTF8(0, 15, ligne); 

  // ==========================================
  // LIGNE 2 : Température centrale
  // ==========================================
  u8g2.setFont(u8g2_font_helvB24_tf); 
  if (sondeValide && temperatureInitialisee) {
    char t[12];
    dtostrf(temperatureC, 1, 1, t);
    snprintf(ligne, sizeof(ligne), "%sC", t);
  } else {
    snprintf(ligne, sizeof(ligne), "--.-C");
  }
  int largeur = u8g2.getUTF8Width(ligne);
  u8g2.drawUTF8((128 - largeur) / 2, 58, ligne); 

  // ==========================================
  // LIGNE 3 : Moteur
  // ==========================================
  u8g2.setFont(u8g2_font_9x15_tf); 
  if (moteurDemande || moteurActif) snprintf(ligne, sizeof(ligne), "Vm: %3u", vitessePct);
  else snprintf(ligne, sizeof(ligne), "Vm: ---");
  u8g2.drawUTF8(0, 90, ligne);

  // Fleche placee juste apres la valeur Vm.
  if (moteurDemande || moteurActif) {
    int xFleche = u8g2.getUTF8Width(ligne) + 2;
    if (xFleche > 78) xFleche = 78;
    if (sensFwd) u8g2.drawTriangle(xFleche, 82, xFleche, 92, xFleche + 6, 87);
    else         u8g2.drawTriangle(xFleche + 6, 82, xFleche + 6, 92, xFleche, 87);
  }
  u8g2.drawVLine(88, 78, 15); 
  u8g2.drawUTF8(96, 90, (moteurDemande || moteurActif) ? "ON " : "OFF");

  // ==========================================
  // LIGNE 4 : Séparation complète de bout en bout
  // ==========================================
  u8g2.drawHLine(0, 102, 128); 

  // ==========================================
  // LIGNE 5 : Fil
  // ==========================================
  snprintf(ligne, sizeof(ligne), "Fil: %-3s         ", filIn ? "In" : "Out"); 
  u8g2.drawUTF8(0, 122, ligne); 

  u8g2.drawVLine(88, 110, 15); 
  u8g2.drawUTF8(96, 122, filSurveillance ? "ON " : "OFF");
}

void dessinerMenu() {
  char val[20];
  
  u8g2.setFont(u8g2_font_9x15_tf);
  u8g2.drawUTF8(0, 13, " MENU GENERAL");  u8g2.drawHLine(0, 17, 128);

  u8g2.setFont(u8g2_font_7x14_tf);
  for (uint8_t i = 0; i < NB_ITEMS; ++i) {
    int y = 32 + i * 14;
    if (i == selectionMenu) {
      u8g2.drawBox(0, y - 10, 128, 13);
      u8g2.setDrawColor(0);
    }
    u8g2.drawUTF8(2, y, nomsMenu[i]);

     val[0] = '\0';
    if (i == ITEM_CHAUFFE) snprintf(val, sizeof(val), "%s", chauffeOn ? "ON" : "OFF");
    if (i == ITEM_CONSIGNE) snprintf(val, sizeof(val), "%d°C", consigneTemp);
    if (i == ITEM_MOTEUR) snprintf(val, sizeof(val), "%s", moteurDemande ? "ON" : "OFF");
    if (i == ITEM_FIL) snprintf(val, sizeof(val), "%s", filSurveillance ? "ON" : "OFF");
    if (val[0]) {
      int w = u8g2.getUTF8Width(val);
      u8g2.drawUTF8(122 - w, y, val);
    }
    u8g2.setDrawColor(1);
  }
  
  u8g2.drawHLine(0, 115, 128);
  u8g2.setFont(u8g2_font_5x8_tf);
  dessinerFlecheHaut(4,120); dessinerFlecheBas(12,120);
  u8g2.drawUTF8(19,127,"Nav");
  dessinerFlecheDroite(48,123); u8g2.drawUTF8(57,127,"Sel");
  dessinerFlecheGauche(81,123); u8g2.drawUTF8(90,127,"Retour");
}

void dessinerEditionTemp() {
  char val[16];
  u8g2.setFont(u8g2_font_7x14_tf);
  u8g2.drawUTF8(0, 14, "  CONSIGNE CHAUFFE");
  u8g2.drawHLine(0, 19, 128);
  
  u8g2.setFont(u8g2_font_9x15_tf);
  u8g2.drawUTF8(0, 38, "  Temp cible");
  
  snprintf(val, sizeof(val), "%d C", consigneTemp);
  u8g2.setFont(u8g2_font_helvB24_tf);
  int w = u8g2.getUTF8Width(val);
  u8g2.drawUTF8((128 - w) / 2, 75, val);
  
  u8g2.drawHLine(0, 95, 128);
  u8g2.setFont(u8g2_font_5x8_tf);
  // X- diminue, X+ augmente ; bouton central valide ; Y- revient au menu.
  dessinerFlecheBas(5,106); u8g2.drawUTF8(14,110,"Dim");
  dessinerFlecheHaut(42,106); u8g2.drawUTF8(51,110,"Aug");
  dessinerBoutonCentral(84,106); u8g2.drawUTF8(93,110,"Valider");
  dessinerFlecheGauche(23,121); u8g2.drawUTF8(32,127,"Retour menu");
}

void dessinerAutotune() {
  char s[32],t[12],p[10],i[10],d[10];
  u8g2.setFont(u8g2_font_7x14_tf);u8g2.drawUTF8(0,14,"    AUTOTUNE PID");u8g2.drawHLine(0,19,128);
  dtostrf(temperatureC,1,1,t);snprintf(s,sizeof(s),"Mesure:%s C",(sondeValide&&temperatureInitialisee)?t:"--.-");u8g2.drawUTF8(3,35,s);
  if(atEcran==AT_UI_PREPARATION){
    snprintf(s,sizeof(s),"Cible: %d C",(int)atCible);u8g2.drawUTF8(3,51,s);
    u8g2.drawUTF8(3,67,atEepromValide?"PID EEPROM valide":"EEPROM vide/invalide");
    if(atEepromValide){dtostrf(atEepromKP,1,1,p);dtostrf(atEepromKI,1,2,i);dtostrf(atEepromKD,1,1,d);snprintf(s,sizeof(s),"P%s I%s D%s",p,i,d);u8g2.drawUTF8(2,84,s);}else u8g2.drawUTF8(3,84,"PID actifs conserves");
    u8g2.drawUTF8(3,103,"Lancer autotune ?");
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawUTF8(14,116,"OUI"); u8g2.drawUTF8(75,116,"NON");
    if(atChoixOui) dessinerFlecheHaut(5,111); else dessinerFlecheBas(66,111);
    dessinerFlecheGauche(4,123); u8g2.drawUTF8(13,127,"Choix");
    dessinerBoutonCentral(61,123); u8g2.drawUTF8(70,127,"Valider");
    dessinerFlecheGauche(83,123); u8g2.drawUTF8(92,127,"Retour");
  }else if(atEcran==AT_UI_RUN){
    snprintf(s,sizeof(s),"Cible: %d C",(int)atCible);u8g2.drawUTF8(3,51,s);snprintf(s,sizeof(s),"Cycles: %u/5",atCycles);u8g2.drawUTF8(3,68,s);u8g2.drawUTF8(3,91,atChauffe?"Chauffage: ON":"Chauffage: OFF");
    u8g2.setFont(u8g2_font_5x8_tf); dessinerBoutonCentral(12,119); u8g2.drawUTF8(21,123,"Annuler");
    dessinerFlecheGauche(79,119); u8g2.drawUTF8(88,123,"Retour");
  }else{
    dtostrf(atAncienKP,1,1,p);dtostrf(atAncienKI,1,2,i);dtostrf(atAncienKD,1,1,d);snprintf(s,sizeof(s),"A P%s I%s D%s",p,i,d);u8g2.drawUTF8(2,53,s);
    dtostrf(atNouveauKP,1,1,p);dtostrf(atNouveauKI,1,2,i);dtostrf(atNouveauKD,1,1,d);snprintf(s,sizeof(s),"N P%s I%s D%s",p,i,d);u8g2.drawUTF8(2,69,s);
    if(atEcran==AT_UI_SAUVEGARDE){
      u8g2.drawUTF8(2,91,"Sauver nouveaux PID ?");
      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.drawUTF8(14,107,"OUI");u8g2.drawUTF8(75,107,"NON");
      if(atChoixOui) dessinerFlecheHaut(5,102); else dessinerFlecheBas(66,102);
      dessinerFlecheGauche(4,119);u8g2.drawUTF8(13,123,"Choix");
      dessinerBoutonCentral(61,119);u8g2.drawUTF8(70,123,"Valider");
      dessinerFlecheGauche(96,119);u8g2.drawUTF8(105,123,"Non");
    } else {
      u8g2.drawUTF8(2,94,autotune==AT_ECHEC?"Autotune en echec":(atEepromVerifiee?"EEPROM: sauvegarde OK":(atSauvegardeRefusee?"Non sauvegarde":"PID actifs inchanges")));
      u8g2.setFont(u8g2_font_5x8_tf);dessinerBoutonCentral(26,119);u8g2.drawUTF8(35,123,"Retour menu");
    }
  }
}

void dessinerFlecheHaut(uint8_t x,uint8_t y){u8g2.drawTriangle(x,y,x-3,y+5,x+3,y+5);}
void dessinerFlecheBas(uint8_t x,uint8_t y){u8g2.drawTriangle(x,y+5,x-3,y,x+3,y);u8g2.drawVLine(x,y,5);}
void dessinerFlecheGauche(uint8_t x,uint8_t y){u8g2.drawTriangle(x,y,x+5,y-3,x+5,y+3);u8g2.drawHLine(x,y,5);}
void dessinerFlecheDroite(uint8_t x,uint8_t y){u8g2.drawTriangle(x+5,y,x,y-3,x,y+3);u8g2.drawHLine(x,y,5);}
void dessinerBoutonCentral(uint8_t x,uint8_t y){u8g2.drawCircle(x,y,3);u8g2.drawDisc(x,y,1);}
void dessinerGestionMoteur(){
  char val[20];u8g2.setFont(u8g2_font_7x14_tf);u8g2.drawUTF8(0,14,"   GESTION MOTEUR");u8g2.drawHLine(0,19,128);
  u8g2.drawUTF8(4,35,"Etat :");u8g2.drawUTF8(55,35,moteurDemande?"ON":"OFF");
  u8g2.drawUTF8(4,51,"Sens :");u8g2.drawUTF8(55,51,sensFwd?"FWD":"REV");if(sensFwd)dessinerFlecheDroite(88,46);else dessinerFlecheGauche(88,46);
  u8g2.drawUTF8(4,67,"Vm :");snprintf(val,sizeof(val),"%u %%",vitessePct);u8g2.drawUTF8(55,67,val);
  u8g2.drawUTF8(4,83,"RPM max:");snprintf(val,sizeof(val),"%u tr/min",(unsigned)MAX_RPM_MOTEUR);u8g2.drawUTF8(65,83,val);
  if(filSurveillance&&!filIn)u8g2.drawUTF8(6,99,"FIL OUT : BLOQUE");else if(arretFilMemorise)u8g2.drawUTF8(6,99,"FIL IN : DEBLOQUE");
  u8g2.drawHLine(0,105,128);u8g2.setFont(u8g2_font_5x8_tf);
  dessinerFlecheHaut(5,112);u8g2.drawUTF8(12,116,"Vm+");dessinerFlecheBas(34,112);u8g2.drawUTF8(41,116,"Vm-");
  dessinerFlecheDroite(68,112);u8g2.drawUTF8(73,116,"FWD");dessinerFlecheGauche(5,124);u8g2.drawUTF8(10,127,"REV");
  dessinerBoutonCentral(49,124);u8g2.drawUTF8(55,127,"Marche/Arret");
}

void rafraichirEcran() {
  if (ecranEnVeille) return;
  unsigned long maintenant = millis();
  if (!ecranSale && maintenant - derniereMajEcran < PERIODE_ECRAN_MS) return;
  derniereMajEcran = maintenant;
  ecranSale = false;

  u8g2.clearBuffer();
  if (modeUI == UI_ACCUEIL) dessinerAccueil();
  else if (modeUI == UI_MENU) dessinerMenu();
  else if (modeUI == UI_EDIT_TEMP) dessinerEditionTemp();
  else if (modeUI == UI_AUTOTUNE) dessinerAutotune();
  else dessinerGestionMoteur();
  unsigned long debutEnvoi = micros();
  u8g2.sendBuffer();
  unsigned long dureeEnvoi = micros() - debutEnvoi;
  totalEcranUs += dureeEnvoi;
  if (dureeEnvoi > dureeEcranMaxUs) dureeEcranMaxUs = dureeEnvoi;
  if (++nbMesuresEcran >= 20) {
    if (debugActif) {
      Serial.print(F("[OLED] sendBuffer moyen=")); Serial.print(totalEcranUs / nbMesuresEcran);
      Serial.print(F(" us max=")); Serial.print(dureeEcranMaxUs);
      Serial.println(F(" us (20 trames)"));
    }
    totalEcranUs = 0; dureeEcranMaxUs = 0; nbMesuresEcran = 0;
  }
}

void gererVeilleEcran() {
  unsigned long maintenant = millis();
  if (!ecranEnVeille && modeUI != UI_AUTOTUNE &&
      maintenant - derniereActivite >= VEILLE_ECRAN_MS) {
    modeUI = UI_ACCUEIL;
    u8g2.setPowerSave(1);
    ecranEnVeille = true;
    activiteAvantVeille = derniereActivite;
  } else if (ecranEnVeille && derniereActivite != activiteAvantVeille) {
    u8g2.setPowerSave(0);
    ecranEnVeille = false;
    ecranSale = true;
  }
}

void gererDebugSerie() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'D' || c == 'd') {
      debugActif = true;
      Serial.println(F("[DEBUG] ON"));
    } else if (c == 'O' || c == 'o') {
      debugActif = false;
      Serial.println(F("[DEBUG] OFF"));
    } else if (c == '?') {
      Serial.println(F("D=debug ON, O=debug OFF, ?=aide"));
    }
  }
}

void imprimerDebug(){
  static unsigned long precedent=0;unsigned long maintenant=millis();if(!debugActif||maintenant-precedent<PERIODE_DEBUG_MS)return;precedent=maintenant;
  Serial.print(F("[ETAT] T="));if(sondeValide&&temperatureInitialisee)Serial.print(temperatureC,1);else Serial.print(F("INVALIDE"));
  Serial.print(F("C Tc="));Serial.print(consigneTemp);Serial.print(F(" Chauffe="));Serial.print(chauffeOn?F("ON"):F("OFF"));
  Serial.print(F(" PWM="));Serial.print((int)pwmChauffe);Serial.print(F(" Moteur="));Serial.print(moteurDemande?F("ON"):F("OFF"));
  Serial.print(F(" Sens="));Serial.print(sensFwd?F("FWD"):F("REV"));Serial.print(F(" Vm="));Serial.print(vitessePct);Serial.print(F("% RPM="));Serial.print(rpmActuel,1);
  Serial.print(F(" Fil="));Serial.print(filIn?F("In"):F("Out"));Serial.print(F(" Surveillance="));Serial.print(filSurveillance?F("ON"):F("OFF"));
  Serial.print(F(" UI="));Serial.print(modeUI);Serial.print(F(" AT="));Serial.print(autotune);
  if(modeUI==UI_AUTOTUNE||autotune!=AT_INACTIF){Serial.print(F(" AT_UI="));Serial.print(atEcran);Serial.print(F(" cible="));Serial.print(atCible,1);Serial.print(F(" cycles="));Serial.print(atCycles);
    Serial.print(F(" ancien="));Serial.print(atAncienKP,2);Serial.print(',');Serial.print(atAncienKI,3);Serial.print(',');Serial.print(atAncienKD,2);
    Serial.print(F(" nouveau="));Serial.print(atNouveauKP,2);Serial.print(',');Serial.print(atNouveauKI,3);Serial.print(',');Serial.print(atNouveauKD,2);
    Serial.print(F(" EEPROM="));Serial.print(atEepromVerifiee?F("VERIFIEE"):(atSauvegardeRefusee?F("REFUSEE"):F("EN_ATTENTE")));
  }Serial.println();
}

void setup() {
  pinMode(PIN_HEATER, OUTPUT);
  digitalWrite(PIN_HEATER, LOW);

  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_DIR, DIR_FWD_LEVEL);  digitalWrite(PIN_EN, HIGH);

  pinMode(PIN_FIL, INPUT_PULLUP);
  pinMode(PIN_JOY_SW, INPUT_PULLUP);

  Serial.begin(115200);

  chargerPID();
  chargerConfig();
  digitalWrite(PIN_DIR, sensFwd ? DIR_FWD_LEVEL : DIR_REV_LEVEL);
  Wire.begin();
  u8g2.begin();
  u8g2.setBusClock(400000);
  u8g2.setContrast(32);

  filIn = false;
  derniereActivite = millis();
  ecranSale = true;

  if (debugActif) {
    Serial.println();
    Serial.println(F("=== BANC HOTEND / MOTEUR - SH1107 128x128 ==="));
  }
}

void loop() {
  gererDebugSerie();
  lireFil();

  if (filSurveillance && FIL_ARRET_MOTEUR_SI_OUT && !filIn &&
      (moteurDemande || moteurActif)) {
    moteurDemande = false;
    arretFilMemorise = true;
    arreterMoteurPhysiquement();
    if (debugActif) Serial.println(F("[FIL] Out : moteur coupe."));
    ecranSale = true;
  }

  gererChauffage();
  gererRampeMoteur();
  gererInterface();
  sauverConfigSiNecessaire();
  gererVeilleEcran();
  rafraichirEcran();
  imprimerDebug();
}