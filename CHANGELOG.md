# Journal des évolutions — Recreator PET

Toutes les évolutions du firmware doivent être consignées ici. Les commits et branches sont conservés afin de permettre de retrouver exactement le code testé.

## Cycle de vie

- **Stable** : version de référence utilisée pour les tests courants.
- **Bêta** : version de développement, non garantie pour l'utilisation prolongée.
- **Candidate (RC)** : bêta dont les évolutions sont terminées, en attente de validation finale.
- **Stable publiée** : uniquement après compilation réussie et validation des essais matériels prévus.

## Versions

### v1.0 — Base stable

- Date de référence : commit du 10 novembre 2026.
- Commit : [`04ce9c963250110d03f63175aa68167be5849839`](https://github.com/SounMayon/RecreatorMega2560/commit/04ce9c963250110d03f63175aa68167be5849839)
- Branche : `stable/v1.0`
- Source : version explicitement testée par l'utilisateur.
- Résultat rapporté : moteur opérationnel dans les deux sens et arrêts conformes aux essais effectués.
- Limites : ne pas extrapoler ces essais à une validation complète de la chauffe, de l'autotune ou de toutes les conditions de sécurité.

### v1.1-beta — Branche de développement initialisée

- Branche : `beta/1.1`
- Base : commit `04ce9c9`.
- Statut : branche créée, code initial identique à v1.0.
- But : reprendre les évolutions UI / veille OLED / menus moteur / gestion filament et traiter le redémarrage constaté dans le commit `67d5805`.
- Aucun changement fonctionnel n'est encore intégré à cette bêta.

## Régression connue

Le commit `67d5805e3d2ca95b18eac2d05308f51706d5e763` est le premier commit identifié par les essais utilisateur comme déclenchant le redémarrage au lancement du moteur par rapport au commit fonctionnel `04ce9c9`. Ce commit ajoute notamment le watchdog AVR 2 secondes (`wdt_enable(WDTO_2S)`, `wdt_reset()`) et des changements d'interface/veille.

Le redémarrage est rapporté dans quatre configurations : moteur branché ou débranché, alimentation moteur branchée ou débranchée. Cause exacte non confirmée. Ne pas affirmer que le watchdog est la cause tant que la piste n'a pas été vérifiée par instrumentation et essais.

## Fiche de validation par version

Pour chaque bêta, enregistrer :
- SHA du commit testé et date du test ;
- carte/clone Mega 2560, RAMPS, driver, alimentation et moteur utilisés ;
- résultat de compilation CI ;
- démarrage / arrêt moteur ;
- FWD et REV ;
- changement de sens en cours de fonctionnement ;
- arrêt filament pendant marche et inversion en attente ;
- joystick et navigation de chaque écran ;
- veille / réveil OLED et actualisation température ;
- persistance EEPROM après redémarrage ;
- mesure de température à froid et à plusieurs paliers ;
- autotune uniquement sous surveillance et avec protection thermique indépendante ;
- éventuels resets et message série de diagnostic.

Un test non réalisé doit être noté « non testé », jamais « validé ».

## Historique des commits importants

| Commit | Description | Observation |
|---|---|---|
| `04ce9c9` | Référence de départ | Test utilisateur : fonctionnement moteur correct |
| `67d5805` | Ajout watchdog et veille OLED | Régression de redémarrage moteur rapportée |
| `75555af` | Documentation thermistance B3950 | README uniquement ; sketch inchangé par rapport au parent |
| `9faaea1` à `24298ca` | Évolutions interface, joystick et sécurité filament | Non promues en stable ; à examiner / retester sur bêta |

