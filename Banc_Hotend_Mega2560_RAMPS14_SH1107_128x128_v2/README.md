# Banc Hotend + Moteur — Mega 2560 / RAMPS 1.4

Projet Arduino pour piloter un hotend, un moteur pas-à-pas et un écran OLED SH1107 128x128.

## Ouvrir le projet

Ouvrir le dossier `Banc_Hotend_Mega2560_RAMPS14_SH1107_128x128_v2` dans l'IDE Arduino, ou ouvrir directement le fichier `Banc_Hotend_Mega2560_RAMPS14_SH1107_128x128_v2.ino`. Le nom du dossier et celui du fichier principal correspondent, comme l'exige l'IDE Arduino.

## Matériel et câblage

- Arduino Mega 2560 + RAMPS 1.4.
- Alimentation 12 V adaptée à la cartouche chauffante et au moteur.
- Driver DRV8825 en position E0 : STEP D26, DIR D28, ENABLE D24 (actif à LOW).
- Chauffage hotend sur D10.
- Thermistance hotend sur T0 / A13 : thermistance 100 kΩ, Beta 3950, pull-up RAMPS 4,7 kΩ (à confirmer avec la référence réelle de la sonde).
- Contact filament NO entre D3 et GND, entrée `INPUT_PULLUP` : LOW = Fil In, HIGH = Fil Out.
- Joystick : X=A5, Y=A9, bouton D40.
- OLED Grove SH1107 128x128 : SDA D20, SCL D21.

**Sécurité électrique :** ne jamais appliquer 12 V sur la broche 5V du Mega. Conserver les protections thermiques matérielles indépendantes du firmware : fusible adapté, coupure thermique indépendante et câblage dimensionné. Vérifier polarités, connecteurs et refroidissement du régulateur avant mise sous tension.

## Navigation

- Accueil : joystick Y+ ouvre le menu ; appui long (1,5 s) ouvre la gestion moteur.
- Gestion moteur : X+ / X- règle la vitesse ; Y+ demande FWD ; Y- demande REV ; appui court marche/arrêt ; appui long retour.
- Menu : X+ / X- navigue ; Y+ ou appui court sélectionne ; Y- revient à l'accueil.
- Consigne : X+ augmente, X- diminue ; appui court valide ; Y- revient.
- Autotune : choix, lancement, annulation et confirmation de sauvegarde PID sur l'écran.

## Paramètres persistants

Les coefficients PID sont enregistrés à l'adresse EEPROM 0. Les paramètres utilisateur (consigne, vitesse, sens, surveillance filament) sont enregistrés séparément à l'adresse 32, avec version et CRC. Les écritures de configuration sont différées après la dernière modification pour éviter les écritures répétées à chaque mouvement du joystick.

## Autotune et limites

L'autotune utilise la consigne courante, un plafond relatif `min(cible + 20 °C, TEMP_SECURITE_MAX - 10 °C)`, et vérifie qu'une commande PWM fixe de 180/255 produit une montée d'au moins 2 °C en 60 s. Ces seuils doivent être validés sur le montage réel. Le contrôle logiciel ne remplace pas un thermostat/fusible thermique matériel.

## Écran et bus I2C

Le bus U8g2 est configuré à 400 kHz. La durée réelle de `sendBuffer()` est mesurée et rapportée sur le port série toutes les 20 trames lorsque le debug est actif. La période de rafraîchissement est de 300 ms. L'écran passe en veille après 60 s d'inactivité.

## Compilation

La CI compile pour Arduino Mega 2560 (ATmega2560) avec Arduino AVR core et U8g2 2.35.30. La compilation CI ne valide pas le comportement thermique, la qualité de mesure de la thermistance, les temps I2C réels ni les mouvements mécaniques : ces points nécessitent des essais matériels prudents.
