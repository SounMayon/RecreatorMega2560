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
#include <stdio.h>

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
// Avec INPUT_PULLUP et un switch NO entre D3 et GND : presse=LOW.
const bool FIL_PRESSE_EST_LOW = true;
const bool FIL_ARRET_MOTEUR_SI_OUT = true;

// Sens logique : adapter si le sens mecanique FWD/REV est inverse.
const uint8_t DIR_FWD_LEVEL = LOW;
const uint8_t DIR_REV_LEVEL = HIGH;

// Micro-pas DRV8825 : verifier que M0/M1/M2 correspondent bien a 1/32.
const float PAS_PAR_TOUR = 200.0f * 32.0f;

// Limite de depart. 100% correspond a MAX_RPM_MOTEUR.
// Ne pas augmenter avant validation mecanique et thermique.
const uint16_t MAX_RPM_MOTEUR = 120;
const uint8_t VITESSE_DEPART_PCT = 50;
const uint8_t PAS_VITESSE_PCT = 5;
const float RPM_DEMARRAGE = 8.0f;
const float ACCEL_RPM_PAR_SECONDE = 120.0f;

// Thermistance 100k B3950 et pull-up RAMPS standard 4,7k.
const float R_PULLUP = 4700.0f;
const float R_THERM_25 = 100000.0f;
const float BETA_THERM = 3950.0f;
const float TEMP_SECURITE_MAX = 290.0f;
const int TEMP_CONSIGNE_MIN = 50;
const int TEMP_CONSIGNE_MAX = 280;

// PID de depart : valeurs a valider sur le hotend reel.
const float KP = 22.0f;
const float KI = 1.08f;
const float KD = 114.0f;
const float BANDE_PID = 10.0f;
const unsigned long PERIODE_RUNAWAY_MS = 40000UL;
const float MONTEE_MIN_RUNAWAY = 2.0f;

#define DEBUG_AU_DEMARRAGE 1
const unsigned long PERIODE_DEBUG_MS = 1000UL;
const unsigned long PERIODE_ECRAN_MS = 150UL;
const unsigned long APPUI_LONG_MS = 1500UL;
const unsigned long FIL_ANTI_REBOND_MS = 30UL;

// ---------------- Etats ----------------
enum ErreurChauffe : uint8_t { ERR_AUCUNE, ERR_SONDE, ERR_TEMP_MAX, ERR_RUNAWAY };
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

bool filSurveillance = false;
bool filIn = false; // etat filtre : presse = In ; relache = Out
bool filInitialise = false;

enum ModeUI : uint8_t { UI_ACCUEIL, UI_MENU, UI_EDIT_TEMP, UI_GESTION_MOTEUR };
ModeUI modeUI = UI_ACCUEIL;

enum ItemMenu : uint8_t {
  ITEM_CHAUFFE,
  ITEM_CONSIGNE,
  ITEM_MOTEUR,
  ITEM_FIL,
  ITEM_RETOUR,
  NB_ITEMS
};
const char* const nomsMenu[NB_ITEMS] = {
  "Chauffage", "Consigne T", "Gestion moteur", "Surveillance Fil", "Retour"
};
uint8_t selectionMenu = 0;

bool debugActif = DEBUG_AU_DEMARRAGE;
bool ecranSale = true;
unsigned long derniereMajEcran = 0;
unsigned long derniereActivite = 0;

// ---------------- Timer4 : impulsions STEP ----------------
// Mega 2560 : D26 = PA4. Pulse HIGH ~3 us.
ISR(TIMER4_COMPA_vect) {
  PORTA |= _BV(PA4);
  __builtin_avr_delay_cycles(48);
  PORTA &= ~_BV(PA4);
}

void timerMoteurDemarrer() {
  noInterrupts();
  TCCR4A = 0;
  TCCR4B = _BV(WGM42) | _BV(CS41); // CTC, horloge 2 MHz
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
  // Arret utilisateur avec deceleration geree par gererRampeMoteur().
  moteurDemande = false;
  ecranSale = true;
}

void appliquerSens(bool nouveauSensFwd) {
  if (nouveauSensFwd == sensFwd) return;

  bool redemarrer = moteurDemande && moteurActif;
  // Securite : arret complet avant de modifier DIR.
  arreterMoteurPhysiquement();
  sensFwd = nouveauSensFwd;
  digitalWrite(PIN_DIR, sensFwd ? DIR_FWD_LEVEL : DIR_REV_LEVEL);
  delayMicroseconds(10);

  if (debugActif) {
    Serial.print(F("[MOTEUR] Nouveau sens : "));
    Serial.println(sensFwd ? F("FWD") : F("REV"));
  }

  if (redemarrer && filAutoriseMoteur()) {
    // La demande reste active : redemarrage automatique dans le nouveau sens.
    demarrerMoteurPhysiquement();
  } else if (redemarrer) {
    moteurDemande = false;
    arretFilMemorise = true;
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
    arretFilMemorise = true;
    arreterMoteurPhysiquement();
    if (debugActif) Serial.println(F("[FIL] Fil=Out : arret moteur."));
    ecranSale = true;
    return;
  }

  float cible = moteurDemande ? rpmCible() : 0.0f;
  float variation = ACCEL_RPM_PAR_SECONDE * dt;

  if (rpmActuel < cible) rpmActuel = min(rpmActuel + variation, cible);
  else if (rpmActuel > cible) rpmActuel = max(rpmActuel - variation, cible);

  if (!moteurDemande && rpmActuel <= RPM_DEMARRAGE) {
    arreterMoteurPhysiquement();
    return;
  }
  reglerFrequenceMoteur(rpmActuel);
}

// ---------------- Chauffage / mesure thermistance ----------------
bool lireTemperature(float &tempC) {
  uint32_t somme = 0;
  for (uint8_t i = 0; i < 16; ++i) somme += analogRead(PIN_THERM);
  float adc = somme / 16.0f;

  if (adc < 3.0f || adc > 1020.0f) return false;

  // RAMPS standard : 4,7k vers +5V, NTC vers GND.
  float resistance = R_PULLUP * adc / (1023.0f - adc);
  float invKelvin = 1.0f / 298.15f
                  + log(resistance / R_THERM_25) / BETA_THERM;
  tempC = (1.0f / invKelvin) - 273.15f;
  return isfinite(tempC) && tempC > -30.0f && tempC < 400.0f;
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
  }
}

void gererChauffage() {
  static unsigned long precedent = 0;
  static unsigned long debutControleRunaway = 0;
  static float temperatureReference = 0.0f;

  unsigned long maintenant = millis();
  if (maintenant - precedent < 200) return;
  float dt = (precedent == 0) ? 0.2f : (maintenant - precedent) / 1000.0f;
  precedent = maintenant;

  float mesure;
  sondeValide = lireTemperature(mesure);
  if (!sondeValide) {
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

  // Surtemperature surveillee meme avec chauffe OFF (MOSFET potentiellement colle).
  if (temperatureC >= TEMP_SECURITE_MAX) {
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

  if (!chauffeOn) {
    couperChauffage();
    integralePID = 0.0f;
    deriveeFiltree = 0.0f;
    temperaturePrecedente = temperatureC;
    temperatureReference = temperatureC;
    debutControleRunaway = maintenant;
    return;
  }

  float erreur = consigneTemp - temperatureC;

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

// ---------------- Switch Fil NO ----------------
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

// ---------------- Joystick : directions et repetiteur ----------------
enum Direction : uint8_t { DIR_NONE, X_PLUS, X_MOINS, Y_PLUS, Y_MOINS };

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
  // Les axes X se repetent pour le reglage; Haut/Bas restent ponctuels.
  if ((actuelle == X_PLUS || actuelle == X_MOINS) &&
      (long)(maintenant - prochaineRepetition) >= 0) {
    prochaineRepetition = maintenant + 130;
    return actuelle;
  }
  return DIR_NONE;
}

// Evenements bouton : appui court au relachement, appui long des 1,5 s.
enum EvenementBouton : uint8_t { BTN_AUCUN, BTN_COURT, BTN_LONG };
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

// ---------------- Interface / menu ----------------
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
  switch (selectionMenu) {
    case ITEM_CHAUFFE:
      activerChauffage();
      break;
    case ITEM_CONSIGNE:
      modeUI = UI_EDIT_TEMP;
      break;
    case ITEM_MOTEUR:
      modeUI = UI_GESTION_MOTEUR;
      break;
    case ITEM_FIL:
      filSurveillance = !filSurveillance;
      if (debugActif) {
        Serial.print(F("[FIL] Surveillance "));
        Serial.println(filSurveillance ? F("ON") : F("OFF"));
      }
      if (filSurveillance && !filIn && moteurActif) {
        moteurDemande = false;
        arretFilMemorise = true;
        arreterMoteurPhysiquement();
      }
      break;
    case ITEM_RETOUR:
      modeUI = UI_ACCUEIL;
      break;
  }
  ecranSale = true;
}

void ajusterVitesse(int variation) {
  int nouvelle = (int)vitessePct + variation;
  nouvelle = constrain(nouvelle, 5, 100);
  vitessePct = (uint8_t)nouvelle;
  if (debugActif) {
    Serial.print(F("[MOTEUR] Vitesse demandee "));
    Serial.print(vitessePct);
    Serial.println(F("%"));
  }
  ecranSale = true;
}

void gererInterface() {
  uint8_t d = lireDirectionEvenement();
  uint8_t b = lireEvenementBouton();
  if (d != DIR_NONE || b != BTN_AUCUN) derniereActivite = millis();

  switch (modeUI) {
    case UI_ACCUEIL:
      if (d == Y_PLUS) {
        modeUI = UI_MENU;
        selectionMenu = 0;
        ecranSale = true;
      }
      if (b == BTN_LONG) {
        modeUI = UI_GESTION_MOTEUR;
        ecranSale = true;
      }
      break;

    case UI_GESTION_MOTEUR:
      if (d == X_PLUS) ajusterVitesse(PAS_VITESSE_PCT);
      else if (d == X_MOINS) ajusterVitesse(-PAS_VITESSE_PCT);
      else if (d == Y_PLUS) appliquerSens(true);
      else if (d == Y_MOINS) appliquerSens(false);

      if (b == BTN_COURT) {
        if (moteurDemande) {
          demanderArretMoteur();
          if (debugActif) Serial.println(F("[MOTEUR] Arret manuel."));
        } else if (filAutoriseMoteur()) {
          moteurDemande = true;
          arretFilMemorise = false;
          if (debugActif) Serial.println(F("[MOTEUR] Marche demandee."));
        } else {
          if (debugActif) Serial.println(F("[MOTEUR] Bloque : Fil=Out."));
        }
        ecranSale = true;
      }
      if (b == BTN_LONG) {
        modeUI = UI_ACCUEIL;
        ecranSale = true;
      }
      break;

    case UI_MENU:
      // X+ = monter, X- = descendre.
      if (d == X_PLUS) {
        selectionMenu = (selectionMenu + NB_ITEMS - 1) % NB_ITEMS;
        ecranSale = true;
      } else if (d == X_MOINS) {
        selectionMenu = (selectionMenu + 1) % NB_ITEMS;
        ecranSale = true;
      }
      // Y+ ou bouton = selectionner.
      if (d == Y_PLUS || b == BTN_COURT) selectionnerMenu();
      // Y- = retour direct a l'accueil.
      if (d == Y_MOINS) {
        modeUI = UI_ACCUEIL;
        ecranSale = true;
      }
      break;

    case UI_EDIT_TEMP:
      if (d == X_PLUS) {
        consigneTemp = min(consigneTemp + 1, TEMP_CONSIGNE_MAX);
        ecranSale = true;
      } else if (d == X_MOINS) {
        consigneTemp = max(consigneTemp - 1, TEMP_CONSIGNE_MIN);
        ecranSale = true;
      }
      if (b == BTN_COURT) {
        modeUI = UI_MENU;
        ecranSale = true;
      }
      if (d == Y_MOINS) {
        modeUI = UI_MENU;
        ecranSale = true;
      }
      break;
  }
}

// ---------------- Dessin OLED ----------------
void dessinerFleche(int x, int y, bool droite) {
  if (droite) {
    u8g2.drawHLine(x, y, 12);
    u8g2.drawLine(x + 7, y - 5, x + 12, y);
    u8g2.drawLine(x + 7, y + 5, x + 12, y);
  } else {
    u8g2.drawHLine(x, y, 12);
    u8g2.drawLine(x + 5, y - 5, x, y);
    u8g2.drawLine(x + 5, y + 5, x, y);
  }
}

void dessinerAccueil() {
  char ligne[24];

  // ==========================================
  // LIGNE 1 : Tc (13 caractéristiques max -> police 9x15, x=0 à 117px)
  // ==========================================
  u8g2.setFont(u8g2_font_9x15_tf); // Largeur 9px fixe, hauteur ~15px
  if (erreurChauffe != ERR_AUCUNE) {
    const char* defaut = (erreurChauffe == ERR_SONDE) ? "ERR SONDE" :
                         (erreurChauffe == ERR_TEMP_MAX) ? "ERR TEMP MAX" : "ERR RUNAWAY";
    snprintf(ligne, sizeof(ligne), "%s", defaut);
  } else if (chauffeOn) {
    snprintf(ligne, sizeof(ligne), "Tc:%3d | On", consigneTemp);
  } else {
    snprintf(ligne, sizeof(ligne), "Tc:--- | OFF");
  }
  u8g2.drawUTF8(0, 15, ligne); // Aligné à gauche, y=15 pour laisser de l'espace au-dessus


  // ==========================================
  // LIGNE 2 : Température centrale (6 caractères -> police massive)
  // ==========================================
  u8g2.setFont(u8g2_font_inb24_mf); // Largeur ~20px par chiffre, hauteur 24px (très visible)
  if (sondeValide && temperatureInitialisee) {
    char t[12];
    dtostrf(temperatureC, 1, 1, t);
    snprintf(ligne, sizeof(ligne), "%sC", t);
  } else {
    snprintf(ligne, sizeof(ligne), "--.-C");
  }
  int largeur = u8g2.getUTF8Width(ligne);
  u8g2.drawUTF8((128 - largeur) / 2, 58, ligne); // Centrage parfait au milieu de la zone


  // ==========================================
  // LIGNE 3 : Moteur (16 caractères max -> police 8x13, x=0 à 128px)
  // ==========================================
  u8g2.setFont(u8g2_font_8x13_tf); // Largeur 8px fixe, hauteur ~13px
  
  // Remplissage complet de la ligne avec formatage strict à 16 caractères
  if (moteurDemande || moteurActif) {
    snprintf(ligne, sizeof(ligne), "Vm:%3u           ", vitessePct); // Les espaces comblent la ligne
  } else {
    snprintf(ligne, sizeof(ligne), "Vm:---           ");
  }
  u8g2.drawUTF8(0, 90, ligne); // Aligné au pixel 0

  // Superposition de la flèche et du statut sur la police monospacée
  if (moteurDemande || moteurActif) {
    dessinerFleche(64, 85, sensFwd); // Recentré à x=64 pour un écran 128px
  }
  u8g2.drawVLine(88, 78, 15); // Ligne verticale séparatrice déplacée à x=88 (11 chars * 8px = 88px)
  u8g2.drawUTF8(96, 90, (moteurDemande || moteurActif) ? "ON " : "OFF");


  // ==========================================
  // LIGNE 4 : Séparation de bout en bout (0 à 128 px)
  // ==========================================
  u8g2.drawHLine(0, 102, 128); // Exploite les 128 pixels de large


  // ==========================================
  // LIGNE 5 : Fil (16 caractères max -> police 8x13, x=0 à 128px)
  // ==========================================
  // Utilisation de la même police 8x13 déjà chargée pour économiser la mémoire flash
  snprintf(ligne, sizeof(ligne), "Fil:%-3s         ", filIn ? "In" : "Out"); // %-3s force 3 caractères de large à gauche
  u8g2.drawUTF8(0, 122, ligne); // Aligné tout en bas (y=122, garde 6px de marge sous la police)

  u8g2.drawVLine(88, 110, 15); // Aligné exactement sur la ligne verticale du dessus (x=88)
  u8g2.drawUTF8(96, 122, filSurveillance ? "ON " : "OFF");
}


void dessinerMenu() {
  char val[20];
  u8g2.setFont(u8g2_font_7x13B_tf);
  u8g2.drawUTF8(3, 13, "MENU");
  u8g2.drawHLine(0, 18, 128);

  u8g2.setFont(u8g2_font_6x12_tf);
  for (uint8_t i = 0; i < NB_ITEMS; ++i) {
    int y = 37 + i * 17;
    if (i == selectionMenu) {
      u8g2.drawBox(0, y - 12, 128, 15);
      u8g2.setDrawColor(0);
    }
    u8g2.drawUTF8(4, y, nomsMenu[i]);

    val[0] = '\0';
    if (i == ITEM_CHAUFFE) snprintf(val, sizeof(val), "%s", chauffeOn ? "ON" : "OFF");
    if (i == ITEM_CONSIGNE) snprintf(val, sizeof(val), "%dC", consigneTemp);
    if (i == ITEM_MOTEUR) snprintf(val, sizeof(val), "%s", moteurDemande ? "ON" : "OFF");
    if (i == ITEM_FIL) snprintf(val, sizeof(val), "%s", filSurveillance ? "ON" : "OFF");
    if (val[0]) {
      int w = u8g2.getUTF8Width(val);
      u8g2.drawUTF8(124 - w, y, val);
    }
    u8g2.setDrawColor(1);
  }
  u8g2.drawHLine(0, 121, 128);
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawUTF8(2, 127, "◄/►:haut/bas ▲:OK ▼:retour");
}

void dessinerEditionTemp() {
  char val[16];
  u8g2.setFont(u8g2_font_7x13B_tf);
  u8g2.drawUTF8(2, 16, "CONSIGNE CHAUFFE");
  u8g2.drawHLine(0, 22, 128);
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawUTF8(2, 47, "Temperature cible");
  snprintf(val, sizeof(val), "%d C", consigneTemp);
  u8g2.setFont(u8g2_font_helvB18_tf);
  int w = u8g2.getUTF8Width(val);
  u8g2.drawUTF8((128 - w) / 2, 82, val);
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawUTF8(2, 111, "◄/► : modifier");
  u8g2.drawUTF8(2, 125, "◉ : valider  ▼ : retour");
}

void dessinerGestionMoteur() {
  char val[20];
  u8g2.setFont(u8g2_font_7x13B_tf);
  u8g2.drawUTF8(2, 14, "GESTION MOTEUR");
  u8g2.drawHLine(0, 20, 128);

  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawUTF8(2, 39, "Etat:");
  u8g2.drawUTF8(53, 39, moteurDemande ? "ON" : "OFF");

  u8g2.drawUTF8(2, 58, "Sens:");
  u8g2.drawUTF8(53, 58, sensFwd ? "FWD  ->" : "REV  <-");

  u8g2.drawUTF8(2, 77, "Vitesse:");
  snprintf(val, sizeof(val), "%u %%", vitessePct);
  u8g2.drawUTF8(72, 77, val);

  u8g2.drawUTF8(2, 96, "RPM limite:");
  snprintf(val, sizeof(val), "%u", MAX_RPM_MOTEUR);
  u8g2.drawUTF8(91, 96, val);

  if (filSurveillance && !filIn) {
    u8g2.drawUTF8(2, 110, "BLOQUE : FIL OUT");
  } else if (arretFilMemorise) {
    u8g2.drawUTF8(2, 110, "Fil libere : relancer");
  }

  u8g2.drawHLine(0, 115, 128);
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawUTF8(2, 121, "◄/► : vitesse  ▲ : FWD");
  u8g2.drawUTF8(2, 127, "▼ : REV  ◉ : ON/OFF");
}

void rafraichirEcran() {
  unsigned long maintenant = millis();
  if (!ecranSale && maintenant - derniereMajEcran < PERIODE_ECRAN_MS) return;
  derniereMajEcran = maintenant;
  ecranSale = false;

  u8g2.clearBuffer();
  if (modeUI == UI_ACCUEIL) dessinerAccueil();
  else if (modeUI == UI_MENU) dessinerMenu();
  else if (modeUI == UI_EDIT_TEMP) dessinerEditionTemp();
  else dessinerGestionMoteur();
  u8g2.sendBuffer();
}

// ---------------- Debug serie ----------------
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

void imprimerDebug() {
  static unsigned long precedent = 0;
  unsigned long maintenant = millis();
  if (!debugActif || maintenant - precedent < PERIODE_DEBUG_MS) return;
  precedent = maintenant;

  Serial.print(F("[ETAT] T="));
  if (sondeValide && temperatureInitialisee) Serial.print(temperatureC, 1);
  else Serial.print(F("INVALIDE"));
  Serial.print(F("C Tc="));
  Serial.print(consigneTemp);
  Serial.print(F(" Chauffe="));
  Serial.print(chauffeOn ? F("ON") : F("OFF"));
  Serial.print(F(" PWM="));
  Serial.print((int)pwmChauffe);
  Serial.print(F(" Moteur="));
  Serial.print(moteurDemande ? F("ON") : F("OFF"));
  Serial.print(F(" Sens="));
  Serial.print(sensFwd ? F("FWD") : F("REV"));
  Serial.print(F(" Vm="));
  Serial.print(vitessePct);
  Serial.print(F("% RPM="));
  Serial.print(rpmActuel, 1);
  Serial.print(F(" Fil="));
  Serial.print(filIn ? F("In") : F("Out"));
  Serial.print(F(" Surveillance="));
  Serial.print(filSurveillance ? F("ON") : F("OFF"));
  Serial.print(F(" UI="));
  Serial.println(modeUI);
}

// ---------------- Setup / loop ----------------
void setup() {
  // Mise en securite des sorties avant initialisation.
  pinMode(PIN_HEATER, OUTPUT);
  digitalWrite(PIN_HEATER, LOW);

  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_DIR, DIR_FWD_LEVEL);
  digitalWrite(PIN_EN, HIGH);

  pinMode(PIN_FIL, INPUT_PULLUP);
  pinMode(PIN_JOY_SW, INPUT_PULLUP);

  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(100000);
  u8g2.begin();
  u8g2.setContrast(32);

  sensFwd = true;
  filIn = false;
  derniereActivite = millis();
  ecranSale = true;

  if (debugActif) {
    Serial.println();
    Serial.println(F("=== BANC HOTEND / MOTEUR - SH1107 128x128 ==="));
    Serial.println(F("Fil NO : presse=In, relache=Out."));
    Serial.println(F("Gestion moteur : clic marche/arret, appui long sortie."));
    Serial.println(F("D=debug ON, O=debug OFF, ?=aide"));
  }
}

void loop() {
  gererDebugSerie();
  lireFil();

  // Si Fil surveillance est ON et que le contact est relache (Out),
  // l'arret moteur est prioritaire et memorise.
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
  rafraichirEcran();
  imprimerDebug();
}
