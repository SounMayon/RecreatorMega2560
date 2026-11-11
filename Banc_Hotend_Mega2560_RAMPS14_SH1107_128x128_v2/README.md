# Banc Hotend + moteur — Version stable v1.0

**Statut : référence stable de départ — essais matériels rapportés par l'utilisateur.**

Cette version est figée sur le commit de référence `04ce9c9` (SHA complet ci-dessous). Elle sert de base de repli et ne doit recevoir aucune expérimentation. Toute évolution se fait sur une branche bêta distincte.

## Référence de version

- Version : **v1.0**
- Branche : `stable/v1.0`
- Commit source : [`04ce9c9`](https://github.com/SounMayon/RecreatorMega2560/commit/04ce9c963250110d03f63175aa68167be5849839)
- SHA complet : `04ce9c963250110d03f63175aa68167be5849839`
- Firmware : `Banc_Hotend_Mega2560_RAMPS14_SH1107_128x128_v2.ino`
- Cette branche conserve le code du commit de référence ; seul ce README décrit le statut de version.

## Règles de gestion des versions

1. `stable/v1.0` est une branche de référence : pas de modifications expérimentales.
2. Les évolutions sont développées sur `beta/1.1`, puis, si nécessaire, sur `beta/1.2`, etc.
3. Une bêta n'est pas déclarée stable sur la seule base d'une compilation réussie.
4. Une promotion en version stable intervient seulement après validation logicielle et essais matériels consignés.
5. Chaque évolution doit avoir un commit distinct, un message descriptif, une entrée de journal et les résultats de tests associés.
6. Ne jamais écraser ni déplacer le commit source de v1.0 ; créer une nouvelle version stable (v1.1, v2.0...) au moment de la promotion.

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

## Navigation et fonctions

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

## Compilation et validation

La CI compile pour Arduino Mega 2560 (ATmega2560) avec Arduino AVR core et U8g2 2.35.30. La compilation ne valide pas le comportement thermique, la qualité de mesure de la thermistance, les temps I2C réels ni les mouvements mécaniques : ces points nécessitent des essais matériels prudents.

### Essais rapportés

L'utilisateur a testé le commit `04ce9c9` : le moteur fonctionne dans les deux sens et s'arrête comme prévu dans les différents cas essayés. Le commit `67d5805`, qui ajoute notamment le watchdog, provoque un redémarrage au lancement du moteur. Le défaut est rapporté avec ou sans moteur branché et avec ou sans alimentation moteur branchée.

Ces essais constituent un retour matériel utilisateur, pas une campagne de qualification exhaustive. Chauffage, thermistance, autotune et protections thermiques indépendantes doivent être validés séparément.

## Journal des versions

| Version | Branche / commit | Statut |
|---|---|---|
| v1.0 | `stable/v1.0` — `04ce9c9` | Base stable de référence selon essais utilisateur |
| v1.1-beta | `beta/1.1` — initialisée depuis `04ce9c9` | En développement ; aucune évolution intégrée à ce stade |

Voir aussi le [journal global des évolutions](../../CHANGELOG.md).
