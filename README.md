# ESP32-CAM — détection ArUco ultra légère

Pipeline de vision embarquée pour détecter les tags **ArUco valeur 13** (pierres Eurobot / Eurobot Junior) sur ESP32-CAM.

L’objectif n’est **pas** de recopier OpenCV : dictionnaire ArUco, `solvePnP`, Hough, etc. sont volontairement exclus. Les décisions (cadrage, ID, pose) restent des algos maison, très légers en CPU et en RAM.

---

## Contexte

| Élément              | Choix                                                              |
| -------------------- | ------------------------------------------------------------------ |
| Capteur              | ESP32-CAM AI-Thinker, QVGA **320×240** niveaux de gris             |
| Cible                | Tag ArUco **13** (4 orientations) sur les faces des pierres        |
| Taille réelle du tag | **7 cm** de côté                                                   |
| Lien série           | Trame binaire fixe 21 octets (`0xAA`, type, id, 4 coins, checksum) |
| Type pierre          | `'P'` ; inconnu `'I'`                                              |

---

## Architecture

```
esp_cam_access_point/
├── esp_cam_access_point.ino   # Setup Wi‑Fi AP / boucle vidéo
├── picture.cpp / picture.h    # Caméra + pipeline de détection
├── Access_point.cpp / .h      # Page HTML (seuil + flux)
└── pc_clone/
    ├── aruco_algo.py / ihm.py # Clone Python (IHM stats)
    └── host/                  # Même picture.cpp compilé sur PC (DirectShow)
```

**Deux modes ESP**

1. **Wi‑Fi AP** (`WIFI true`) : softAP + serveur HTTP (page `:80`, flux MJPEG `:81`).
2. **Sans Wi‑Fi** : `video_handler()` en boucle (série / debug).

**Simulateur PC** : compile `picture.cpp` avec des stubs Arduino/ESP, capture webcam DirectShow, même logique de détection.

---

## Fonctionnalités déjà en place

### Vision / détection

- [x] Capture QVGA grayscale, buffers en PSRAM, grab « latest »
- [x] Seuillage **linéaire** sur le gris (`pixel < seuil`)
- [x] Ouverture morphologique **3×3** (réduit le bruit noir)
- [x] Effacement du fond noir connecté au **bord du cadre** (préserve les îlots ArUco)
- [x] Bandes via histogramme horizontal des noirs
- [x] Contour de Moore + 4 droites (ACP) + intersections → quadrilatère
- [x] Filtres géométriques : convexité, ratio des côtés, taille min/max
- [x] Lecture ID grille 6×6 (4×4 utile), Hamming &lt; 2
- [x] Table tag 13 : `10767`, `43192`, `61524`, `7445` (0° / 90° / 180° / 270°)
- [x] Validation **bordure noire** (~80 % de la couronne)
- [x] Repère 2D (X rouge, Y vert) + angle de pose image
- [x] Profondeur approximative (côté 70 mm, focale ~280 px en QVGA)
- [x] Affichage distance en cm sur le flux

### Interface & outillage

- [x] SoftAP + page web avec **curseur de seuil** (port 80)
- [x] Flux MJPEG traité (port 81) pour ne pas bloquer le réglage
- [x] Commandes série (`O`/`C` LED, `I` capture SD)
- [x] Simulateur C++ PC (`pc_clone/host/lancer_cpp.bat`)
- [x] Clone Python + IHM stats mémoire / temps (`pc_clone/`)
- [x] Essai pleine résolution **uniquement** côté simulateur PC



---

## Démarrage rapide

### ESP32-CAM (Arduino)

1. Ouvrir le sketch `esp_cam_access_point`.
2. Régler `WIFI`, `ssid`, `password` dans `.ino`.
3. Flasher la carte AI-Thinker.
4. Se connecter au Wi‑Fi AP, ouvrir `http://192.168.4.1/`.
5. Ajuster le seuil ; le flux est sur le **port 81**.



---

## Paramètres utiles (`picture.cpp`)

| Constante                     | Rôle (ordre de grandeur)          |
| ----------------------------- | --------------------------------- |
| `SEUIL_NOIR` / `seuil_noir`   | Seuil gris (défaut 100)           |
| `TAILLE_MIN` / `TAILLE_MAX`   | Taille bbox acceptée (px)         |
| `CONTOUR_MAX`                 | Longueur max du contour           |
| `RATIO_COTES`                 | Perspective max côté long / court |
| `COTE_ARUCO_MM` / `FOCALE_PX` | Conversion taille → distance      |

La trame série **ne doit pas changer** de format (21 octets) tant que le robot côté réception n’est pas mis à jour.

---

## Roadmap

### Court terme — robustesse & allègement

| Priorité | Idée                          | Objectif                                                                                                                                                               |
| -------- | ----------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| P0       | **Filtre temporel / spatial** | Si un ArUco est vu en *(x,y,taille)*, à l’image suivante ne scanner qu’une ROI autour ; si une tache noire cohérente y reste → réutiliser pose / ID avec peu de calcul |
| P0       | **Prédiction simple**         | Garder vitesse image du centroïde (Δx, Δy) pour centrer la ROI                                                                                                         |
| P1       | **Perte de piste**            | Après N frames sans confirmation → rescan full frame                                                                                                                   |

### Moyen terme — exposition & seuil

| Priorité | Idée                                     | Objectif                                                                                                                         |
| -------- | ---------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------- |
| P1       | **Calibration auto (méthode à définir)** | Au **démarrage** + quand la **luminosité globale** change fortement (ex. moyenne / percentiles bougent au-delà d’un seuil)       |
| P1       | **Mesure de déclenchement**              | Histogramme ou moyenne glissante ; hystérésis pour éviter de recalibrer en continu                                               |
| P2       | **Seuil adaptatif par zones**            | Grille (ex. 4×3) : un seuil local par tuile, appliqué seulement dans la tuile — plus sûr qu’un adaptatif global                  |
| P2       | **Couplage ROI + seuil local**           | Une fois la piste active, affiner le seuil **uniquement** dans la ROI (idée déjà explorée ; à reprendre avec le filtre temporel) |

### Longer terme — produit robot

| Priorité | Idée                                     | Objectif                                                 |
| -------- | ---------------------------------------- | -------------------------------------------------------- |
| P2       | Multi-pierres stables (jusqu’à 4 pistes) | Eurobot : plusieurs faces / pierres                      |
| P2       | Fusion avec odométrie / IMU              | Réduire les faux positifs en mouvement                   |
| P3       | Mode « match » vs « debug »              | Debug = flux + overlays ; match = trames série seulement |
| P3       | Tuning focale / distance sur terrain     | Recaler `FOCALE_PX` avec mesures réelles                 |
| P3       | Watchdog mémoire / watchdog caméra       | Redémarrage propre si `fb_get` échoue                    |

---

## Idées à exploiter (bonnes pistes)

1. **Cascade coarse → fine**  
   Full frame basse résolution (ou sous-échantillonné ×2) pour trouver des candidats, puis traitement QVGA natif seulement dans les ROI. Gros gain CPU.

2. **Score de confiance**  
   Combiner inliers des côtés + qualité bordure + écart Hamming. N’envoyer la trame `'P'` qu’au-dessus d’un score ; sinon garder en piste « faible ».

3. **Verrouillage d’orientation**  
   Une fois l’orientation 0/90/180/270 connue, ne tester que celle-ci (+ voisines) tant que la piste vit.

4. **Masque de mouvement**  
   Différence frame-à-frame très légère : ignorer les zones stables non trackées (décor), concentrer le scan sur le changement + les ROI actives.

5. **Calibration « deux points »**  
   Au boot : viser une scène avec du noir et du blanc connus (tag ou charte) ; régler seuil (et éventuellement ae/brightness) pour maximiser contraste bordure / fond — plus simple qu’un balayage exhaustif.

6. **Seuil par bande Y**  
   Variante ultra légère du seuil zonal : un seuil par bande horizontale déjà utilisée par `histY`, sans vraie grille 2D.

7. **Journal de réglages**  
   Sur SD : seuil + ae + luminosité moyenne + nb détections / minute pour rejouer les essais terrain.

8. **Contrainte compétition**  
   Garder une build « match » sans HTTP/MJPEG pour libérer CPU et éviter les sockets ouverts.

---

## Principes de conception (à respecter)

- **Ultra léger** : chaque feature doit justifier son coût en cycles / RAM.
- **Pas d’OpenCV pour l’ID** : capture/affichage PC OK ; identification = table + Hamming maison.
- **Trame série stable** : compatibilité robot / PC de réception.
- **Firmware et simulateur alignés** : une seule source de vérité (`picture.cpp`).
- **Échecs passés = leçons** : pas de seuillage adaptatif global naïf ; pas de calib qui force contraste/expo au max.

---

## État actuel (résumé)

| Domaine                            | État                                             |
| ---------------------------------- | ------------------------------------------------ |
| Détection tag 13 + pose + distance | Opérationnel, à durcir en conditions réelles     |
| Réglage seuil (web + PC)           | Opérationnel                                     |
| Filtre temporel / ROI              | **À faire** (priorité roadmap)                   |
| Calibration auto                   | **À définir** (déclencheurs OK, méthode ouverte) |
| Seuil par zones                    | **Idée** (après filtre temporel)                 |

---

## Licence / usage

Projet pédagogique / robotique compétition. Adapter SSID, mots de passe et constantes optique avant usage terrain.
