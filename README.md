# RecreatorMega2560

Firmware de banc de test hotend + moteur pour Arduino Mega 2560 / RAMPS 1.4.

## Versions du firmware

- **Stable v1.0** : branche [`stable/v1.0`](https://github.com/SounMayon/RecreatorMega2560/tree/stable/v1.0), figée sur le commit testé `04ce9c9`.
- **Bêta v1.1** : branche [`beta/1.1`](https://github.com/SounMayon/RecreatorMega2560/tree/beta/1.1), initialisée depuis v1.0 pour les prochaines évolutions.
- Les expérimentations se font exclusivement en bêta. Une version stable n'est promue qu'après validation logicielle et essais matériels consignés.

## Documentation

- [Documentation technique et câblage v1.0](Banc_Hotend_Mega2560_RAMPS14_SH1107_128x128_v2/README.md)
- [Journal des versions et des tests](CHANGELOG.md)
- [Firmware v1.0](Banc_Hotend_Mega2560_RAMPS14_SH1107_128x128_v2/Banc_Hotend_Mega2560_RAMPS14_SH1107_128x128_v2.ino)

## Compilation et sécurité

La CI compile pour Arduino Mega 2560 / ATmega2560 avec Arduino AVR core et U8g2 2.35.30. Une compilation réussie ne remplace pas les essais matériels ni une protection thermique indépendante.
