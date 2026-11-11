# Banc Hotend + moteur — Arduino Mega 2560 / RAMPS 1.4

Firmware de banc de test pour piloter un hotend, un moteur pas-à-pas et un écran OLED Grove SH1107 128×128.

> **Sécurité :** ce firmware ne remplace pas une protection thermique matérielle indépendante. Conserver un fusible/coupe-circuit thermique adapté, une alimentation correctement dimensionnée et un câblage approprié. Ne jamais injecter le 12 V sur la broche 5 V du Mega.

## 1. Matériel

- Arduino Mega 2560 (ATmega2560)
- RAMPS 1.4 alimentée en 12 V
- Driver DRV8825 en position E0
- Cartouche chauffante et thermistance montées sur le bloc hotend
- Switch de détection filament normalement ouvert (NO)
- Joystick analogique avec bouton
- OLED Grove 1.12" SH1107G 128×128 (carte V2.1)

## 2. Câblage

| Fonction | RAMPS / Mega | Détail |
|---|---:|---|
| DRV8825 E0 STEP | D26 | Impulsions, pilotées par Timer4 |
| DRV8825 E0 DIR | D28 | FWD = LOW, REV = HIGH |
| DRV8825 E0 ENABLE | D24 | Actif à LOW |
| Chauffage hotend | D10 | Sortie MOSFET hotend |
| Thermistance hotend | T0 / A13 | Entrée analogique |
| Switch filament | D3 | Contact NO entre D3 et GND |
| Joystick X | A5 | Axe horizontal |
| Joystick Y | A9 | Axe vertical |
| Bouton joystick | D40 | Bouton vers GND, entrée pull-up |
| OLED SDA | D20 | I2C SDA |
| OLED SCL | D21 | I2C SCL |

### Switch filament

Le contact est câblé entre D3 et GND avec `INPUT_PULLUP` :
- Contact fermé / joystick du switch appuyé : **LOW = Fil In**
- Contact ouvert / switch relâché : **HIGH = Fil Out**

La surveillance filament est désactivable dans le menu. Lorsqu’elle est activée, Fil Out provoque un arrêt immédiat du moteur et annule toute inversion de sens en attente. Le redémarrage n’est pas automatique : remettre le filament en position In puis redonner manuellement l’ordre de marche.

### Thermistance

Le firmware utilise T0/A13 et une table d’interpolation nominale conçue pour une thermistance **100 kΩ B3950**, avec pull-up **4,7 kΩ** et ADC 10 bits. La table n’est pas une calibration individuelle. Vérifier la référence réelle de la thermistance et la valeur effective de la pull-up de la RAMPS ; comparer à une sonde de référence correctement positionnée avant les essais à haute température.

## 3. Ouvrir et compiler

Ouvrir le dossier `Banc_Hotend_Mega2560_RAMPS14_SH1107_128x128_v2` dans l’IDE Arduino, ou directement le fichier :

`Banc_Hotend_Mega2560_RAMPS14_SH1107_128x128_v2.ino`

Sélectionner **Arduino Mega 2560** comme carte. La compilation continue est définie dans `.github/workflows/arduino-compile.yml` et épingle U8g2 à la version 2.35.30.

La réussite de la compilation ne constitue pas un essai matériel : chauffage, thermistance, I2C, moteur, switch et protections doivent être validés sur le montage réel.

## 4. Navigation

Les directions sont celles du joystick. L’appui long correspond à environ **1,5 seconde**.

### Accueil

- **Y+** : ouvrir le menu général.
- **Appui long** : ouvrir directement la gestion moteur.

L’accueil affiche la consigne et l’état du chauffage, la température mesurée, la vitesse/direction et l’état du switch filament et de sa surveillance.

### Menu général

- **X+ / X−** : déplacer la sélection.
- **Y+** ou **appui court** : sélectionner.
- **Y−** : retour à l’accueil.

Éléments disponibles : Chauffe, consigne, Moteur, Capteur (surveillance filament), Autotune PID.

### Gestion moteur

- **X+ / X−** : augmenter / diminuer la vitesse par pas de 5 %.
- **Y+** : demander FWD.
- **Y−** : demander REV.
- **Appui court** : marche / arrêt.
- **Appui long** : quitter vers l’écran d’origine (accueil ou menu).

Une inversion demandée pendant que le moteur tourne est effectuée après une rampe de ralentissement et un arrêt. À la sortie par appui long, les commandes directionnelles sont ignorées jusqu’au retour du joystick au neutre : cela évite une activation involontaire de REV.

Si la surveillance filament est active et que Fil Out est détecté, le moteur est coupé sans attendre la rampe et les demandes de changement de sens en attente sont effacées.

### Réglage de consigne

- **X+** : augmenter la consigne.
- **X−** : diminuer la consigne.
- **Appui court** : valider.
- **Y−** : revenir au menu.

Plage logicielle : **50 à 280 °C**.

### Autotune PID

L’autotune utilise la consigne courante. Avant lancement, les coefficients PID EEPROM sont présentés lorsqu’ils sont valides et l’utilisateur confirme le lancement. Après calcul, une confirmation distincte permet de sauvegarder ou non les nouveaux coefficients. Une sauvegarde est relue et vérifiée ; en cas de refus ou d’échec, les anciens coefficients restent actifs en RAM.

Conditions et limites codées :
- Hystérésis de régulation autotune : ±2 °C.
- PWM fixe de test : 180/255.
- Test préliminaire : au moins 2 °C de montée en 60 secondes.
- Plafond : minimum entre cible +20 °C et limite de sécurité −10 °C.
- Temporisation maximale autotune : 12 minutes, 5 cycles attendus.

L’autotune nécessite une surveillance du montage et ne doit pas être lancé sans protections thermiques matérielles indépendantes.

## 5. Réglages principaux du firmware

Les paramètres suivants sont déclarés dans le sketch :

| Paramètre | Valeur | Rôle |
|---|---:|---|
| `MAX_RPM_MOTEUR` | 120 tr/min | Limite logicielle initiale du moteur |
| `VITESSE_DEPART_PCT` | 50 % | Vitesse au démarrage |
| `PAS_VITESSE_PCT` | 5 % | Incrément joystick |
| `RPM_DEMARRAGE` | 8 tr/min | Vitesse minimale de démarrage/transition |
| `ACCEL_RPM_PAR_SECONDE` | 120 | Rampe d’accélération/décélération |
| `TEMP_SECURITE_MAX` | 290 °C | Limite de sécurité logicielle |
| `TEMP_CONSIGNE_MIN` | 50 °C | Consigne minimale |
| `TEMP_CONSIGNE_MAX` | 280 °C | Consigne maximale |
| `VEILLE_ECRAN_MS` | 60 000 ms | Délai avant veille OLED |
| `APPUI_LONG_MS` | 1 500 ms | Durée d’un appui long |

Ne pas augmenter les limites thermiques pour compenser une erreur de mesure. Les valeurs moteur doivent être adaptées progressivement au moteur, au driver, à l’alimentation et à la charge réelle.

## 6. EEPROM

Deux structures indépendantes sont utilisées :
- Adresse **0** : coefficients PID.
- Adresse **32** : configuration utilisateur (consigne, vitesse, sens FWD/REV, surveillance filament).

Les structures contiennent un identifiant, une version et un CRC. La configuration utilisateur est enregistrée après 1,5 seconde sans nouvelle modification, afin d’éviter une écriture à chaque mouvement du joystick.

## 7. OLED et I2C

- Contrôleur : SH1107G, 128×128.
- Pilote U8g2 : `U8G2_SH1107_SEEED_128X128_F_HW_I2C`.
- I2C configuré à **400 kHz**.
- Rafraîchissement normal : **300 ms**.
- Après 60 secondes d’inactivité, l’écran affiche uniquement la température ; sa position change toutes les 15 secondes pour limiter le marquage OLED.
- Une action sur le joystick réveille l’interface.

## 8. Watchdog et diagnostic série

Le watchdog AVR est activé avec un délai de **2 secondes** après l’initialisation matérielle. Il est réarmé à chaque tour normal de la boucle principale ; si le programme se bloque, le microcontrôleur redémarre.

Le port série fonctionne à **115200 bauds** :
- `D` : activer les messages de debug.
- `O` : désactiver les messages de debug.
- `?` : afficher l’aide.

Les mesures de durée d’envoi OLED sont rapportées toutes les 20 trames lorsque le debug est actif.

## 9. Limites et validation

Avant toute utilisation prolongée, vérifier notamment :
1. le sens réel FWD/REV du moteur et sa limite de vitesse ;
2. l’arrêt filament dans toutes les situations, y compris pendant une inversion demandée ;
3. la mesure de température sur plusieurs points avec une sonde de référence ;
4. le fonctionnement de la coupure thermique matérielle indépendante ;
5. le comportement après veille, réveil et redémarrage watchdog.

Le firmware seul ne peut garantir la sécurité d’un montage chauffant. Les essais doivent être réalisés progressivement et sous surveillance.
