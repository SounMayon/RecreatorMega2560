# Banc Hotend + moteur — bêta v1.1

> **Version bêta de validation d'interface.** Cette branche est issue de la base stable v1.0 (`04ce9c9`). Elle intègre les évolutions d'interface, de veille OLED, de navigation joystick et de sécurité filament sélectionnées après cette base. Elle est destinée aux essais et n'est pas déclarée stable.

## Périmètre de cette bêta

**Inclus :**
- affichage de la température pendant la veille OLED, avec rafraîchissement périodique et déplacement du texte ;
- menus moteur dédiés : Marche/Arrêt, Direction et Vitesse ;
- correction des axes X/Y dans les menus moteur et consigne ;
- verrouillage des commandes directionnelles jusqu'au retour du joystick au neutre après sortie du menu moteur ;
- arrêt filament prioritaire, annulation des demandes de marche et d'inversion en attente, resynchronisation du sens demandé ;
- répétition joystick sur les deux axes ;
- documentation du câblage, des réglages, de la navigation et des essais.

**Exclus volontairement :**
- **Watchdog AVR** introduit dans `67d5805` : exclu pour isoler le redémarrage constaté au démarrage moteur.
- **Sélection différée de la vitesse** introduite dans `d69dba3` : exclue. La vitesse reste modifiée immédiatement.

## Plan de validation de la bêta

1. Démarrage, navigation et sortie de tous les menus.
2. Vérifier les axes X/Y dans les menus moteur, direction, vitesse et consigne.
3. Vérifier marche/arrêt, FWD et REV.
4. Vérifier arrêt filament en marche et lors d'une inversion en attente.
5. Vérifier veille OLED, actualisation de température et réveil.
6. Surveiller tout redémarrage ou comportement inattendu.

Noter le SHA exact testé et le résultat de chaque point. Une compilation réussie ne remplace pas les essais matériels. Tant que ces essais ne sont pas consignés, cette bêta n'est pas certifiée.

---

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

## 4. Navigation — bêta v1.1

Les directions correspondent aux axes physiques du joystick : **X = horizontal**, **Y = vertical**. L'appui long correspond à environ **1,5 seconde**.

### Accueil
- **Y+** : ouvrir le menu général.
- **Appui long** : ouvrir directement le menu de gestion moteur.

### Menu général
- **X+ / X−** : déplacer la sélection.
- **Y+** ou **appui court** : sélectionner.
- **Y−** : revenir à l'accueil.
- Lorsque l'élément **Moteur** est sélectionné, **appui long** ouvre directement le menu moteur.

### Menu moteur
- **X+ / X−** : naviguer entre Marche/Arrêt, Direction et Vitesse.
- **Y+** ou **appui court** : entrer dans l'élément sélectionné.
- **Y−** ou **appui long** : revenir au menu général.
- Après la sortie, les directions du joystick sont ignorées jusqu'à son retour au neutre afin d'éviter une commande directionnelle résiduelle.

### Marche / Arrêt
- Sélectionner **Marche / Arrêt** avec Y+ ou le bouton.
- **Y+** ou **appui court** : basculer la demande marche/arrêt.
- Le verrouillage filament reste prioritaire.

### Choix de direction
- **X+ / X−** : basculer entre FWD et REV.
- **Y+** ou **appui court** : appliquer le choix.
- **Y−** : revenir sans appliquer.

### Réglage de vitesse — immédiat dans cette bêta
- **X+ / X−** : augmenter / diminuer la vitesse par pas de 5 %.
- **Y−** ou **appui long** : revenir au menu moteur.
- **Y+** ou **appui court** : quitter l'écran de vitesse.
- La modification est immédiate ; il n'y a volontairement pas de sélection différée ni de validation séparée dans cette version.

### Réglage de consigne
- **X+** : augmenter la consigne.
- **X−** : diminuer la consigne.
- **Y+** ou **appui court** : valider.
- **Y−** ou **appui long** : annuler et revenir au menu.

### Autotune PID
L'autotune conserve les étapes de préparation, lancement, annulation et confirmation de sauvegarde PID. Il doit être réalisé sous surveillance et avec une protection thermique matérielle indépendante.

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

## 8. Diagnostic série

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

## Thermistance T0 : table nominale 100 kOhm B3950

La conversion utilise une table ADC/température avec interpolation linéaire, calculée pour une thermistance nominale 100 kOhm à 25 °C, Beta 3950 K, et la résistance pull-up RAMPS de 4,7 kOhm. Les points couvrent 0 à 300 °C par pas de 10 °C. La table est stockée en mémoire programme (PROGMEM).

| Température | ADC approximatif |
|---:|---:|
| 20 °C | 986 |
| 50 °C | 905 |
| 100 °C | 611 |
| 150 °C | 305 |
| 200 °C | 140 |
| 220 °C | 104 |
| 250 °C | 68 |
| 280 °C | 46 |
| 290 °C | 41 |

Ces valeurs sont un modèle théorique générique, pas une calibration de la sonde montée sur le hotend. La tolérance de la thermistance, du Beta réel, de la pull-up et de la référence ADC peut entraîner un écart. La table n'améliore pas magiquement la résolution ADC : autour de 250 °C, un pas ADC reste de plusieurs degrés. Avant d'utiliser le banc à haute température, comparer la mesure à une sonde de référence correctement positionnée, vérifier la température ambiante à froid et contrôler la résistance réelle de la pull-up. Ne pas relever TEMP_SECURITE_MAX pour compenser un écart. Toute protection thermique matérielle indépendante doit rester en place.
