# Eye-In-Hand Workflow Guide

Ce workflow sert a faire proprement:

1. generer un ArUco imprimable
2. capturer 20-30 poses robot + camera + marqueur
3. calculer `tool_from_camera`
4. sauver la calibration
5. lancer `target_conf_deterministic.py`

## Important

Le marqueur ArUco sert ici a la calibration `tool_from_camera`.

Apres avoir:

- capture assez de poses
- calcule la calibration
- sauve la calibration dans `tools/eye_in_hand_collector_servo_config.json`

tu peux enlever ce marqueur de calibration.

Mais:

- si tu deplaces la camera sur le joint 6, il faut recalibrer
- si tu modifies la fixation mecanique camera-outil, il faut recalibrer
- si tu veux reutiliser une pose collecteur deja verrouillee, le collecteur doit rester au meme endroit

## Commande la plus simple

Depuis la racine du workspace:

```bash
python3 eye_in_hand_workflow.py wizard \
  --config tools/eye_in_hand_collector_servo_config.json \
  --samples-json tools/handeye_samples.json \
  --sample-target 25 \
  --min-samples 8 \
  --auto-save-config \
  --launch-after-save
```

## Ce que fait `wizard`

Il genere:

- un PNG du marqueur
- un HTML imprimable a 100%

Puis il ouvre la fenetre de capture/calibration.

## Fichiers generes

- `tools/aruco_marker_id0_230mm.png`
- `tools/aruco_marker_id0_230mm.html`
- `tools/handeye_samples.json`
- mise a jour de `tools/eye_in_hand_collector_servo_config.json`

## Impression du marqueur

1. ouvre le fichier HTML genere
2. imprime a 100% sans "fit to page"
3. mesure le carre a la regle
4. il doit faire exactement la taille demandee dans la config

## Touches pendant la capture

- `c` : capturer la pose courante
- `d` : supprimer la derniere pose
- `k` : calculer la calibration
- `p` : sauver la calibration dans la config
- `l` : lancer `target_conf_deterministic.py`
- `q` : quitter

## Bonnes poses a capturer

Ne prends pas 25 poses presque identiques.

Il faut varier:

- gauche / droite
- haut / bas
- proche / loin
- rotation du poignet
- inclinaison de la camera

## Workflow recommande

1. imprime le marqueur
2. fixe le marqueur a la position du collecteur
3. capture 20 a 30 poses tres differentes
4. appuie sur `k`
5. verifie les valeurs `tool_from_camera_position_m` et `tool_from_camera_rpy_rad`
6. appuie sur `p`
7. retire le marqueur si tu passes ensuite sur un workflow markerless/deterministe
8. lance `target_conf_deterministic.py`

## Commandes separees

### 1. Generer seulement le marqueur

```bash
python3 eye_in_hand_workflow.py generate-aruco \
  --config tools/eye_in_hand_collector_servo_config.json
```

### 2. Capturer et calibrer

```bash
python3 eye_in_hand_workflow.py capture-calibration \
  --config tools/eye_in_hand_collector_servo_config.json \
  --samples-json tools/handeye_samples.json \
  --sample-target 25 \
  --auto-save-config
```

### 3. Lancer la version deterministe

```bash
python3 eye_in_hand_workflow.py launch-deterministic
```
