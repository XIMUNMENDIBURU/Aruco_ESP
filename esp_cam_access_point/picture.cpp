#include "picture.h"
#include "esp_timer.h"
#include <cmath>
#include <cstdlib>
#include "SD_MMC.h"
#include "FS.h"
// ==========================================
// CONFIGURATION FACILE DU SEUILLAGE
// ==========================================
#define SEUIL_NOIR 100       // Seuil linéaire sur le gris brut (0..255). Noir si pixel < seuil.
#define TAILLE_MIN 25        // Taille minimum du carré (pixels)
#define TAILLE_MAX 200       // Prise de près : un tag peut dépasser 120 px en QVGA
#define QUALITE_JPEG 80      // Entre 1 et 100
#define RATIO_COTES 2.5f     // côté long / côté court, laisse passer la perspective
#define INLIER_DIST 2.5f     // un peu plus tolérant sur les bords en escalier
#define INLIER_RATIO 0.50f   // part minimale de points collés à la droite
#define COTE_MIN_PTS 5
#ifndef CONTOUR_MAX
#define CONTOUR_MAX 1200
#endif
#ifndef HIST_Y_MAX
#define HIST_Y_MAX 240
#endif
#define COTE_ARUCO_MM 70
#define FOCALE_PX 280        // 320 px de large, champ horizontal d'environ 60°
#define PISTE_MAX 4
#define DET_MAX 8
#define KALMAN_PERDU_MAX 8   // frames sans mesure avant perte de piste
#define FULLSCAN_PERIOD 12   // full frame périodique même avec pistes
#define ROI_MARGE 1.9f       // ROI = cote * marge (+ sigma)
#define LEDblanche 4
#define canalPWM 7

int seuil_noir = SEUIL_NOIR;
bool capture = false;
bool wifiMode = false;
char pictureName[16] = "IDK";
httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

void init_cam() {
  ledcAttachPin(LEDblanche, 7);
  ledcSetup(canalPWM, 5000, 12);
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("Erreur Montage");
  }
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 24000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QVGA;  // 320x240
  config.jpeg_quality = 10;            // Qualité légèrement meilleure (10-12 est le sweet spot)

  // OPTIMISATION CRITIQUE : Passer à 2 buffers pour le pipeline de traitement
  config.fb_count = 2;

  // Allocation en PSRAM (nécessaire pour fb_count > 1 en QVGA)
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;  // Toujours traiter l'image la plus récente

  // Initialisation Caméra
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    printf("Erreur init caméra 0x%x", err);
    return;
  }

  sensor_t* s = esp_camera_sensor_get();

  // Paramètres d'image pour faciliter la détection du noir
  s->set_brightness(s, -2);  // Un peu plus sombre pour assombrir les zones noires
  s->set_contrast(s, 1);     // Contraste modéré : trop haut écrase les gris utiles
  s->set_sharpness(s, 2);    // Netteté au max pour des bords de carrés propres

  // Gain et AEC (Auto Exposure Control)
  s->set_gain_ctrl(s, 0);  // Désactiver le gain auto pour éviter le bruit blanc dans le noir
  s->set_agc_gain(s, 0);   // Gain manuel au minimum
  s->set_ae_level(s, -1);  // Légère sous-exposition, moins agressive qu'avec le LOG
  // LOG plus utilisé : le seuillage est linéaire sur le gris brut.
}

static uint16_t histY[HIST_Y_MAX];
static uint16_t histX[320];
static int bilan_pierres = 0;
static int bilan_inconnus = 0;
static int largeur_image = 320;

static int echelle_image() {
  int echelle = largeur_image / 320;
  return echelle < 1 ? 1 : echelle;
}

// --- Filtre de Kalman 1D (position + vitesse), un axe ---
struct Filtre1D {
  float x, v;
  float p00, p01, p10, p11;
};

struct Detection {
  char type;
  uint16_t id;
  int orientation;
  int c1x, c1y, c3x, c3y, c4x, c4y, c2x, c2y;
  float cx, cy, cote;
  bool associe;
};

struct Piste {
  bool actif;
  bool vu;
  Filtre1D fx, fy;
  float cote;
  uint16_t id;
  int orientation;
  int perdu;
  int age;
};

static Detection dets[DET_MAX];
static int ndets = 0;
static Piste pistes[PISTE_MAX];
static int frames_depuis_full = 0;
static int64_t t_prev_us = 0;

// Contourne un ICE du GCC Xtensa (postreload) sur certains float ternaires / sqrtf.
#if defined(ARDUINO_ARCH_ESP32)
#define KALMAN_ATTR __attribute__((noinline, optimize("Os")))
#else
#define KALMAN_ATTR
#endif

static KALMAN_ATTR void filtre_init(Filtre1D* f, float x) {
  f->x = x;
  f->v = 0.f;
  f->p00 = 25.f;
  f->p01 = 0.f;
  f->p10 = 0.f;
  f->p11 = 100.f;
}

static KALMAN_ATTR void filtre_predire(Filtre1D* f, float dt, float q) {
  f->x = f->x + f->v * dt;
  float dt2 = dt * dt;
  float a = f->p00;
  float b = f->p01;
  float c = f->p10;
  float d = f->p11;
  f->p00 = a + dt * (b + c) + dt2 * d + q;
  f->p01 = b + dt * d;
  f->p10 = c + dt * d;
  f->p11 = d + q;
}

static KALMAN_ATTR void filtre_corriger(Filtre1D* f, float z, float r) {
  float innov = z - f->x;
  float S = f->p00 + r;
  if (S < 0.001f) S = 0.001f;
  float k0 = f->p00 / S;
  float k1 = f->p10 / S;
  f->x = f->x + k0 * innov;
  f->v = f->v + k1 * innov;
  float a = f->p00;
  float b = f->p01;
  float c = f->p10;
  float d = f->p11;
  f->p00 = (1.f - k0) * a;
  f->p01 = (1.f - k0) * b;
  f->p10 = c - k1 * a;
  f->p11 = d - k1 * b;
}

static float longueur_segment(int ax, int ay, int bx, int by) {
  float dx = (float)(bx - ax);
  float dy = (float)(by - ay);
  return sqrtf(dx * dx + dy * dy);
}

static float detection_cote(int c1x, int c1y, int c3x, int c3y, int c4x, int c4y, int c2x, int c2y) {
  float cote = longueur_segment(c1x, c1y, c3x, c3y);
  cote += longueur_segment(c3x, c3y, c4x, c4y);
  cote += longueur_segment(c4x, c4y, c2x, c2y);
  cote += longueur_segment(c2x, c2y, c1x, c1y);
  return cote * 0.25f;
}

static int piste_libre() {
  for (int i = 0; i < PISTE_MAX; i++) {
    if (!pistes[i].actif) return i;
  }
  return -1;
}

static void piste_creer(const Detection* d) {
  int i = piste_libre();
  if (i < 0) return;
  Piste* p = &pistes[i];
  p->actif = true;
  p->vu = true;
  filtre_init(&p->fx, d->cx);
  filtre_init(&p->fy, d->cy);
  p->cote = d->cote;
  p->id = d->id;
  p->orientation = d->orientation;
  p->perdu = 0;
  p->age = 1;
  Serial.printf("Piste+%d id=%u cx=%d cy=%d s=%d\n", i, (unsigned)d->id, (int)d->cx, (int)d->cy, (int)d->cote);
}

static KALMAN_ATTR void piste_roi(const Piste* p, int w, int h, int* x0, int* y0, int* x1, int* y1) {
  // Marge sans sqrtf (évite ICE GCC ESP32) : approx. via variance elle-même.
  float var = p->fx.p00 + p->fy.p00;
  if (var < 0.f) var = 0.f;
  float marge = ROI_MARGE * p->cote + 0.12f * var + 16.f;
  if (marge < 28.f) marge = 28.f;
  int cx = (int)(p->fx.x + 0.5f);
  int cy = (int)(p->fy.x + 0.5f);
  int m = (int)(marge + 0.5f);
  int xa = cx - m;
  int xb = cx + m;
  int ya = cy - m;
  int yb = cy + m;
  if (xa < 0) xa = 0;
  if (ya < 0) ya = 0;
  if (xb >= w) xb = w - 1;
  if (yb >= h) yb = h - 1;
  *x0 = xa;
  *y0 = ya;
  *x1 = xb;
  *y1 = yb;
}

static KALMAN_ATTR bool dans_gate(const Piste* p, const Detection* d) {
  float dx = d->cx - p->fx.x;
  float dy = d->cy - p->fy.x;
  float dist2 = dx * dx + dy * dy;
  float var = p->fx.p00 + p->fy.p00;
  if (var < 1.f) var = 1.f;
  float rayon = 0.85f * p->cote + 0.35f * var;
  if (rayon < 30.f) rayon = 30.f;
  return dist2 <= (rayon * rayon);
}

static void scanner_zone(camera_fb_t* fb, uint8_t* rgb_buf, int x0, int y0, int x1, int y1) {
  int bande = 6 * echelle_image();
  int y_start = -1;
  for (int y = y0; y <= y1; y++) {
    if (histY[y] > bande && y_start == -1) {
      y_start = y;
    } else if ((histY[y] <= bande || y == y1) && y_start != -1) {
      int yE = y;
      if (y_start < y0) y_start = y0;
      if (yE > y1 + 1) yE = y1 + 1;
      if (yE > y_start) process_y_band(fb, rgb_buf, (uint16_t)y_start, (uint16_t)yE, (uint16_t)x0, (uint16_t)(x1 + 1));
      y_start = -1;
    }
  }
}

void draw_line(uint8_t* buf, int w, int x0, int y0, int x1, int y1) {
  int16_t dx = abs(x1 - x0);     //définition de la distance à parcourir en x
  int16_t dy = -abs(y1 - y0);    //définition de la distance à parcourir en y
  int8_t sx = x0 < x1 ? 1 : -1;  // définition du sens de parcours en x
  int8_t sy = y0 < y1 ? 1 : -1;  // définition du sens de parcours en y
  int16_t err = dx + dy, e2 = 0;
  while (true) {
    uint32_t idx = ((uint32_t)y0 * w + x0) * 3;  // Au pixel suivant !
    buf[idx + 1] = 0;                            // vert
    buf[idx] = 178;                              // bleu
    buf[idx + 2] = 255;                          // rouge
    if (x0 == x1 && y0 == y1) break;             //condition d'arrêt
    e2 = 2 * err;                                // calcul du facteur de compensation
    if (e2 >= dy) {                              // si l'erreur est en suffisament faible partie due au décalage en y par rapport au décalage en x on augmente x
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {  // si l'erreur est en suffisament grande partie au décalage en x par rapport au décalage en y on augmente y
      err += dx;
      y0 += sy;
    }
  }
}

void draw_debug_rect(uint8_t* buf, int w, int x1, int y1, int x2, int y2) {
  for (int x = x1; x <= x2; x++) {  // on trace deux lignes paralllèle suivant l'axe x
    int iH = (y1 * w + x) * 3;
    int iB = (y2 * w + x) * 3;
    buf[iH] = 0;
    buf[iH + 1] = 0;
    buf[iH + 2] = 255;
    buf[iB] = 0;
    buf[iB + 1] = 0;
    buf[iB + 2] = 255;
  }
  for (int y = y1; y <= y2; y++) {  // on trace deux lignes paralllèle suivant l'axe y
    int iG = (y * w + x1) * 3;
    int iD = (y * w + x2) * 3;
    buf[iG] = 0;
    buf[iG + 1] = 0;
    buf[iG + 2] = 255;
    buf[iD] = 0;
    buf[iD + 1] = 0;
    buf[iD + 2] = 255;
  }
}

int gris_du_seuil() {
  return seuil_noir;
}

static void effacer_composante(uint8_t* buf, int w, int h, int x0, int y0);
static void effacer_fond_cadre(uint8_t* buf, int w, int h);

// Ouverture 3x3 sur le noir (0) : enlève les petites taches. Erosion = max, dilatation = min.
static void ouverture_noir_3x3(uint8_t* buf, uint8_t* tmp, int w, int h) {
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      uint8_t m = 0;
      for (int dy = -1; dy <= 1; dy++) {
        int yy = y + dy;
        if (yy < 0) yy = 0;
        if (yy >= h) yy = h - 1;
        const uint8_t* row = buf + yy * w;
        for (int dx = -1; dx <= 1; dx++) {
          int xx = x + dx;
          if (xx < 0) xx = 0;
          if (xx >= w) xx = w - 1;
          if (row[xx] > m) m = row[xx];
        }
      }
      tmp[y * w + x] = m;
    }
  }
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      uint8_t m = 255;
      for (int dy = -1; dy <= 1; dy++) {
        int yy = y + dy;
        if (yy < 0) yy = 0;
        if (yy >= h) yy = h - 1;
        const uint8_t* row = tmp + yy * w;
        for (int dx = -1; dx <= 1; dx++) {
          int xx = x + dx;
          if (xx < 0) xx = 0;
          if (xx >= w) xx = w - 1;
          if (row[xx] < m) m = row[xx];
        }
      }
      buf[y * w + x] = m;
    }
  }
}

void binarize_and_histY(camera_fb_t* fb, uint8_t* rgb_buf, uint16_t* hY) {
  uint16_t w = (uint16_t)fb->width;
  uint16_t h = (uint16_t)fb->height;
  uint8_t* pixel_ptr = fb->buf;
  for (int i = 0; i < (int)w * (int)h; i++) {
    pixel_ptr[i] = (pixel_ptr[i] < (uint8_t)seuil_noir) ? 0 : 255;
  }

  uint8_t* tmp = (uint8_t*)malloc((size_t)w * h);
  if (tmp) {
    ouverture_noir_3x3(fb->buf, tmp, w, h);
    free(tmp);
  }

  // Fond noir touchant le cadre : l'effacer sans toucher aux tags isolés (îlots).
  effacer_fond_cadre(fb->buf, w, h);

  memset(hY, 0, h * sizeof(uint16_t));
  uint8_t* rgb_ptr = rgb_buf;
  pixel_ptr = fb->buf;
  for (uint16_t y = 0; y < h; y++) {
    uint16_t black_count = 0;
    for (uint16_t x = 0; x < w; x++) {
      uint8_t binary = *pixel_ptr++;
      if (binary == 0) black_count++;
      *rgb_ptr++ = binary;
      *rgb_ptr++ = binary;
      *rgb_ptr++ = binary;
    }
    hY[y] = black_count;
  }
}

int profondeur_mm(int c1x, int c1y, int c3x, int c3y, int c4x, int c4y, int c2x, int c2y) {
  float cote = hypotf((float)(c3x - c1x), (float)(c3y - c1y));
  cote += hypotf((float)(c4x - c3x), (float)(c4y - c3y));
  cote += hypotf((float)(c2x - c4x), (float)(c2y - c4y));
  cote += hypotf((float)(c1x - c2x), (float)(c1y - c2y));
  cote *= 0.25f;
  if (cote < 1.f) return 0;
  int focale = FOCALE_PX * largeur_image / 320;
  if (focale < 1) focale = FOCALE_PX;
  return (int)(COTE_ARUCO_MM * (float)focale / cote + 0.5f);
}

uint16_t read_aruco_id(camera_fb_t* fb, uint16_t c1x, uint16_t c1y, uint16_t c3x, uint16_t c3y, uint16_t c4x, uint16_t c4y, uint16_t c2x, uint16_t c2y) {
  const uint8_t GRID = 6;
  uint16_t id_bits = 0;
  uint16_t w = fb->width;

  // Parcourir la grille de haut en bas
  for (int gY = 0; gY < GRID; gY++) {
    for (int gX = 0; gX < GRID; gX++) {

      // 1. Calcul des coordonnées normalisées (u, v) de 0.0 à 1.0
      // On ajoute 0.5f pour taper pile au milieu de la case
      float u = (gX + 0.5f) / (float)GRID;
      float v = (gY + 0.5f) / (float)GRID;

      // 2. Interpolation Bilinéaire pour redresser la perspective
      // On mélange les 4 coins selon la position (u,v) dans la grille
      float HG_mix = (1.0f - u) * (1.0f - v);
      float HD_mix = u * (1.0f - v);
      float BD_mix = u * v;
      float BG_mix = (1.0f - u) * v;

      int px = (int)(HG_mix * c1x + HD_mix * c3x + BD_mix * c4x + BG_mix * c2x);
      int py = (int)(HG_mix * c1y + HD_mix * c3y + BD_mix * c4y + BG_mix * c2y);

      // 3. Extraction du bit (uniquement pour la zone de données interne 4x4)
      // On ignore la bordure noire (index 0 et 5)
      if (gX > 0 && gX < 5 && gY > 0 && gY < 5) {
        if (px >= 0 && px < w && py >= 0 && py < fb->height) {
          // Lecture du pixel binarisé (0 ou 255)
          // On considère blanc si > 128
          bool is_white = (fb->buf[py * w + px] > 128);

          id_bits = (id_bits << 1) | (is_white ? 1 : 0);
        }
      }
    }
  }
  return id_bits;
}

// Au moins 80 % de la couronne 6x6 doit être noire, sinon ce n'est pas un ArUco.
static bool bordure_noire_ok(camera_fb_t* fb, uint16_t c1x, uint16_t c1y, uint16_t c3x, uint16_t c3y, uint16_t c4x, uint16_t c4y, uint16_t c2x, uint16_t c2y) {
  const uint8_t GRID = 6;
  uint16_t w = fb->width;
  int noirs = 0;
  int total = 0;
  for (int gY = 0; gY < GRID; gY++) {
    for (int gX = 0; gX < GRID; gX++) {
      if (gX > 0 && gX < 5 && gY > 0 && gY < 5) continue;
      float u = (gX + 0.5f) / (float)GRID;
      float v = (gY + 0.5f) / (float)GRID;
      float HG_mix = (1.0f - u) * (1.0f - v);
      float HD_mix = u * (1.0f - v);
      float BD_mix = u * v;
      float BG_mix = (1.0f - u) * v;
      int px = (int)(HG_mix * c1x + HD_mix * c3x + BD_mix * c4x + BG_mix * c2x);
      int py = (int)(HG_mix * c1y + HD_mix * c3y + BD_mix * c4y + BG_mix * c2y);
      total++;
      if (px < 0 || py < 0 || px >= w || py >= (int)fb->height) continue;
      if (fb->buf[py * w + px] <= 128) noirs++;
    }
  }
  return total > 0 && noirs * 5 >= total * 4;
}

struct __attribute__((packed)) ArucoFrame {
  uint8_t header = 0xAA;  // Octet de synchronisation (Header)
  uint8_t type;           // 'B' pour bon, 'I' pour inconnu, etc.
  uint16_t id;            // ID Aruco
  uint16_t x[4];          // Les 4 coins X
  uint16_t y[4];          // Les 4 coins Y
  uint8_t checksum;       // Pour vérifier que la trame n'est pas corrompue
};

void send_aruco_frame(uint16_t id, char type, uint16_t* cx, uint16_t* cy) {
  ArucoFrame frame;
  frame.type = type;
  frame.id = id;
  for (int i = 0; i < 4; i++) {
    frame.x[i] = cx[i];
    frame.y[i] = cy[i];
  }

  // Calcul simple d'un checksum (XOR de tous les octets)
  uint8_t* ptr = (uint8_t*)&frame;
  uint8_t cs = 0;
  for (size_t i = 0; i < sizeof(frame) - 1; i++) {
    cs ^= ptr[i];
  }
  frame.checksum = cs;

  // Envoi binaire brut
  Serial.write(ptr, sizeof(frame));
}

bool correspond(uint16_t id, uint16_t target) {
  uint16_t diff = id ^ target;
  uint8_t error_count = 0;
  for (uint8_t i = 0; i < 16; i++) {
    if ((diff >> i) & 1) error_count++;
  }
  return (error_count < 2);
}

static const int8_t DX8[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
static const int8_t DY8[8] = { 0, 1, 1, 1, 0, -1, -1, -1 };
static uint16_t contourX[CONTOUR_MAX];
static uint16_t contourY[CONTOUR_MAX];

struct Droite {
  float nx, ny, c;
};

static int tracer_contour(uint8_t* buf, int w, int h, int x0, int y0, bool* ferme) {
  int x = x0;
  int y = y0;
  int prev = 4;
  int n = 0;
  *ferme = false;
  for (int guard = 0; guard < CONTOUR_MAX && n < CONTOUR_MAX; guard++) {
    int found = -1;
    for (int k = 0; k < 8; k++) {
      int d = (prev + 1 + k) & 7;
      int nx = x + DX8[d];
      int ny = y + DY8[d];
      if (nx >= 0 && ny >= 0 && nx < w && ny < h && buf[ny * w + nx] == 0) {
        found = d;
        break;
      }
    }
    if (found < 0) break;
    contourX[n] = (uint16_t)x;
    contourY[n] = (uint16_t)y;
    n++;
    x += DX8[found];
    y += DY8[found];
    prev = (found + 4) & 7;
    if (x == x0 && y == y0) {
      *ferme = true;
      break;
    }
  }
  return n;
}

static void consommer_contour(uint8_t* buf, int w, int h, int n, bool boite) {
  if (n <= 0) return;
  if (!boite) {
    for (int i = 0; i < n; i++) buf[(int)contourY[i] * w + contourX[i]] = 1;
    return;
  }
  int x0 = contourX[0], x1 = x0, y0 = contourY[0], y1 = y0;
  for (int i = 1; i < n; i++) {
    if (contourX[i] < x0) x0 = contourX[i];
    if (contourX[i] > x1) x1 = contourX[i];
    if (contourY[i] < y0) y0 = contourY[i];
    if (contourY[i] > y1) y1 = contourY[i];
  }
  for (int y = y0; y <= y1; y++) {
    uint8_t* row = buf + y * w;
    for (int x = x0; x <= x1; x++) {
      if (row[x] == 0) row[x] = 1;
    }
  }
}

static bool droite_du_cote(int a, int b, int n, Droite* out) {
  int count = 0;
  double sx = 0;
  double sy = 0;
  int i = a;
  for (int guard = 0; guard <= n; guard++) {
    sx += contourX[i];
    sy += contourY[i];
    count++;
    if (i == b) break;
    i = (i + 1) % n;
  }
  if (count < COTE_MIN_PTS) return false;
  float mx = (float)(sx / count);
  float my = (float)(sy / count);
  float cxx = 0, cyy = 0, cxy = 0;
  i = a;
  for (int guard = 0; guard <= n; guard++) {
    float dx = contourX[i] - mx;
    float dy = contourY[i] - my;
    cxx += dx * dx;
    cyy += dy * dy;
    cxy += dx * dy;
    if (i == b) break;
    i = (i + 1) % n;
  }
  float angle = 0.5f * atan2f(2.0f * cxy, cxx - cyy);
  float tx = cosf(angle);
  float ty = sinf(angle);
  out->nx = -ty;
  out->ny = tx;
  out->c = -out->nx * mx - out->ny * my;
  int proches = 0;
  i = a;
  for (int guard = 0; guard <= n; guard++) {
    float dist = fabsf(out->nx * contourX[i] + out->ny * contourY[i] + out->c);
    if (dist <= INLIER_DIST) proches++;
    if (i == b) break;
    i = (i + 1) % n;
  }
  return ((float)proches / (float)count) >= INLIER_RATIO;
}

static bool intersection(const Droite* d1, const Droite* d2, float* x, float* y) {
  float det = d1->nx * d2->ny - d1->ny * d2->nx;
  if (fabsf(det) < 1e-3f) return false;
  *x = (d1->ny * d2->c - d2->ny * d1->c) / det;
  *y = (d2->nx * d1->c - d1->nx * d2->c) / det;
  return true;
}

static bool quadrilatere_convexe(float x[4], float y[4]) {
  float cx = (x[0] + x[1] + x[2] + x[3]) * 0.25f;
  float cy = (y[0] + y[1] + y[2] + y[3]) * 0.25f;
  int ordre[4] = { 0, 1, 2, 3 };
  for (int i = 1; i < 4; i++) {
    int v = ordre[i];
    float angle = atan2f(y[v] - cy, x[v] - cx);
    int j = i - 1;
    while (j >= 0 && atan2f(y[ordre[j]] - cy, x[ordre[j]] - cx) > angle) {
      ordre[j + 1] = ordre[j];
      j--;
    }
    ordre[j + 1] = v;
  }
  int signe = 0;
  for (int k = 0; k < 4; k++) {
    int i1 = ordre[k];
    int i2 = ordre[(k + 1) & 3];
    int i3 = ordre[(k + 2) & 3];
    float produit = (x[i2] - x[i1]) * (y[i3] - y[i2]) - (y[i2] - y[i1]) * (x[i3] - x[i2]);
    if (fabsf(produit) < 1.0f) continue;
    int courant = produit > 0 ? 1 : -1;
    if (signe == 0) signe = courant;
    else if (courant != signe) return false;
  }
  return true;
}

static uint16_t arrondi_image(float v, int limite) {
  if (v < 0) return 0;
  if (v >= limite) return (uint16_t)(limite - 1);
  return (uint16_t)(v + 0.5f);
}

static bool coins_par_droites(int n, int w, int h, uint16_t* c1x, uint16_t* c1y, uint16_t* c3x, uint16_t* c3y, uint16_t* c4x, uint16_t* c4y, uint16_t* c2x, uint16_t* c2y) {
  if (n < COTE_MIN_PTS * 4) return false;
  int iMinS = 0, iMaxS = 0, iMinD = 0, iMaxD = 0;
  for (int i = 1; i < n; i++) {
    int somme = (int)contourX[i] + (int)contourY[i];
    int diff = (int)contourX[i] - (int)contourY[i];
    if (somme < (int)contourX[iMinS] + (int)contourY[iMinS]) iMinS = i;
    if (somme > (int)contourX[iMaxS] + (int)contourY[iMaxS]) iMaxS = i;
    if (diff < (int)contourX[iMinD] - (int)contourY[iMinD]) iMinD = i;
    if (diff > (int)contourX[iMaxD] - (int)contourY[iMaxD]) iMaxD = i;
  }
  int idx[4] = { iMinS, iMaxS, iMinD, iMaxD };
  for (int a = 0; a < 4; a++) {
    for (int b = a + 1; b < 4; b++) {
      if (idx[a] == idx[b]) return false;
      if (idx[b] < idx[a]) {
        int tmp = idx[a];
        idx[a] = idx[b];
        idx[b] = tmp;
      }
    }
  }
  Droite droites[4];
  for (int k = 0; k < 4; k++) {
    if (!droite_du_cote(idx[k], idx[(k + 1) & 3], n, &droites[k])) return false;
  }
  float px[4], py[4];
  for (int k = 0; k < 4; k++) {
    if (!intersection(&droites[k], &droites[(k + 1) & 3], &px[k], &py[k])) return false;
  }
  if (!quadrilatere_convexe(px, py)) return false;

  int i1 = 0, i4 = 0, i2 = 0, i3 = 0;
  for (int i = 1; i < 4; i++) {
    if (px[i] + py[i] < px[i1] + py[i1]) i1 = i;
    if (px[i] + py[i] > px[i4] + py[i4]) i4 = i;
    if (px[i] - py[i] < px[i2] - py[i2]) i2 = i;
    if (px[i] - py[i] > px[i3] - py[i3]) i3 = i;
  }
  if (i1 == i2 || i1 == i3 || i1 == i4 || i2 == i3 || i2 == i4 || i3 == i4) return false;

  float longueurs[4] = {
    hypotf(px[i1] - px[i3], py[i1] - py[i3]),
    hypotf(px[i3] - px[i4], py[i3] - py[i4]),
    hypotf(px[i4] - px[i2], py[i4] - py[i2]),
    hypotf(px[i2] - px[i1], py[i2] - py[i1]),
  };
  float court = longueurs[0], longu = longueurs[0];
  for (int i = 1; i < 4; i++) {
    if (longueurs[i] < court) court = longueurs[i];
    if (longueurs[i] > longu) longu = longueurs[i];
  }
  if (court < 1.0f || longu > RATIO_COTES * court) return false;

  float minX = px[0], maxX = px[0], minY = py[0], maxY = py[0];
  for (int i = 1; i < 4; i++) {
    if (px[i] < minX) minX = px[i];
    if (px[i] > maxX) maxX = px[i];
    if (py[i] < minY) minY = py[i];
    if (py[i] > maxY) maxY = py[i];
  }
  float largeur = maxX - minX;
  float hauteur = maxY - minY;
  int echelle = echelle_image();
  int tmin = TAILLE_MIN * echelle;
  int tmax = TAILLE_MAX * echelle;
  if (largeur < tmin || hauteur < tmin || largeur >= tmax || hauteur >= tmax) return false;

  *c1x = arrondi_image(px[i1], w);
  *c1y = arrondi_image(py[i1], h);
  *c3x = arrondi_image(px[i3], w);
  *c3y = arrondi_image(py[i3], h);
  *c4x = arrondi_image(px[i4], w);
  *c4y = arrondi_image(py[i4], h);
  *c2x = arrondi_image(px[i2], w);
  *c2y = arrondi_image(py[i2], h);
  return true;
}

static void tracer_segment(uint8_t* buf, int w, int h, int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b) {
  int16_t dx = abs(x1 - x0);
  int16_t dy = -abs(y1 - y0);
  int8_t sx = x0 < x1 ? 1 : -1;
  int8_t sy = y0 < y1 ? 1 : -1;
  int16_t err = dx + dy;
  while (true) {
    if (x0 >= 0 && y0 >= 0 && x0 < w && y0 < h) {
      uint32_t idx = ((uint32_t)y0 * w + x0) * 3;
      buf[idx] = b;
      buf[idx + 1] = g;
      buf[idx + 2] = r;
    }
    if (x0 == x1 && y0 == y1) break;
    int16_t e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
}

static void tracer_fleche(uint8_t* buf, int w, int h, int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b) {
  tracer_segment(buf, w, h, x0, y0, x1, y1, r, g, b);
  float angle = atan2f((float)(y1 - y0), (float)(x1 - x0));
  tracer_segment(buf, w, h, x1, y1, x1 + (int)lroundf(6.0f * cosf(angle + 2.6f)), y1 + (int)lroundf(6.0f * sinf(angle + 2.6f)), r, g, b);
  tracer_segment(buf, w, h, x1, y1, x1 + (int)lroundf(6.0f * cosf(angle - 2.6f)), y1 + (int)lroundf(6.0f * sinf(angle - 2.6f)), r, g, b);
}

static void dessiner_repere(uint8_t* buf, int w, int h, int c1x, int c1y, int c3x, int c3y, int c4x, int c4y, int c2x, int c2y, int orientation) {
  float cx = (c1x + c3x + c4x + c2x) / 4.0f;
  float cy = (c1y + c3y + c4y + c2y) / 4.0f;
  float dx = (float)c3x - (float)c1x;
  float dy = (float)c3y - (float)c1y;
  float rad = orientation * 0.0174532925f;
  float cos_a = cosf(rad);
  float sin_a = sinf(rad);
  float xdx = dx * cos_a - dy * sin_a;
  float xdy = dx * sin_a + dy * cos_a;
  float ydx = -xdy;
  float ydy = xdx;
  float cote = hypotf(dx, dy);
  if (cote < 1.0f) cote = 1.0f;
  float echelle = 0.42f * cote;
  float lx = hypotf(xdx, xdy);
  float ly = hypotf(ydx, ydy);
  if (lx < 1.0f) lx = 1.0f;
  if (ly < 1.0f) ly = 1.0f;
  int ox = (int)lroundf(cx);
  int oy = (int)lroundf(cy);
  tracer_fleche(buf, w, h, ox, oy, (int)lroundf(cx + xdx / lx * echelle), (int)lroundf(cy + xdy / lx * echelle), 255, 40, 40);
  tracer_fleche(buf, w, h, ox, oy, (int)lroundf(cx + ydx / ly * echelle), (int)lroundf(cy + ydy / ly * echelle), 40, 210, 70);
}

static const uint8_t GLYPH_CHIFFRE[10][5] = {
    { 7, 5, 5, 5, 7 }, { 2, 6, 2, 2, 7 }, { 7, 1, 7, 4, 7 }, { 7, 1, 7, 1, 7 }, { 5, 5, 7, 1, 1 },
    { 7, 4, 7, 1, 7 }, { 7, 4, 7, 5, 7 }, { 7, 1, 1, 1, 1 }, { 7, 5, 7, 5, 7 }, { 7, 5, 7, 1, 7 },
};

static void tracer_chiffre(uint8_t* buf, int w, int h, int x, int y, int chiffre) {
  for (int ligne = 0; ligne < 5; ligne++) {
    for (int col = 0; col < 3; col++) {
      if ((GLYPH_CHIFFRE[chiffre][ligne] & (4 >> col)) == 0) continue;
      int px = x + col;
      int py = y + ligne;
      if (px < 0 || py < 0 || px >= w || py >= h) continue;
      uint32_t idx = ((uint32_t)py * w + px) * 3;
      buf[idx] = 0;
      buf[idx + 1] = 200;
      buf[idx + 2] = 255;
    }
  }
}

static void tracer_cm(uint8_t* buf, int w, int h, int x, int y, int mm) {
  int cm = (mm + 5) / 10;
  if (cm < 0) cm = 0;
  if (cm > 999) cm = 999;
  int chiffres[3];
  int n = 0;
  int valeur = cm;
  do {
    chiffres[n++] = valeur % 10;
    valeur /= 10;
  } while (valeur > 0 && n < 3);
  for (int i = n - 1; i >= 0; i--) {
    tracer_chiffre(buf, w, h, x, y, chiffres[i]);
    x += 4;
  }
}

void process_and_draw_aruco(camera_fb_t* fb, uint8_t* rgb_buf, uint16_t xSeed, uint16_t ySeed) {
  uint16_t w = fb->width;
  uint16_t h = fb->height;
  bool ferme = false;
  int n = tracer_contour(fb->buf, w, h, xSeed, ySeed, &ferme);
  if (n == 0) {
    fb->buf[ySeed * w + xSeed] = 1;
    return;
  }
  if (!ferme) {
    // Contour trop long ou ouvert : ne pas flood-fill (risque d'effacer les tags).
    consommer_contour(fb->buf, w, h, n, false);
    return;
  }
  if (n < 4 * max(8, TAILLE_MIN - 2) * echelle_image()) {
    consommer_contour(fb->buf, w, h, n, true);
    return;
  }
  consommer_contour(fb->buf, w, h, n, true);

  uint16_t c1x, c1y, c3x, c3y, c4x, c4y, c2x, c2y;
  if (!coins_par_droites(n, w, h, &c1x, &c1y, &c3x, &c3y, &c4x, &c4y, &c2x, &c2y)) return;
  if (!bordure_noire_ok(fb, c1x, c1y, c3x, c3y, c4x, c4y, c2x, c2y)) return;

  static const uint16_t PIERRE_13[] = { 10767, 43192, 61524, 7445 };
  static const int PIERRE_ORI[] = { 0, 90, 180, 270 };
  uint16_t id = read_aruco_id(fb, c1x, c1y, c3x, c3y, c4x, c4y, c2x, c2y);
  char type = 'I';
  int orientation = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (correspond(id, PIERRE_13[i])) {
      type = 'P';
      orientation = PIERRE_ORI[i];
      break;
    }
  }
  uint16_t x[4] = { c1x, c3x, c4x, c2x };
  uint16_t y[4] = { c1y, c3y, c4y, c2y };
  if (type == 'P') bilan_pierres++;
  else bilan_inconnus++;
  if (ndets < DET_MAX) {
    Detection* d = &dets[ndets++];
    d->type = type;
    d->id = id;
    d->orientation = orientation;
    d->c1x = c1x;
    d->c1y = c1y;
    d->c3x = c3x;
    d->c3y = c3y;
    d->c4x = c4x;
    d->c4y = c4y;
    d->c2x = c2x;
    d->c2y = c2y;
    d->cx = (c1x + c3x + c4x + c2x) * 0.25f;
    d->cy = (c1y + c3y + c4y + c2y) * 0.25f;
    d->cote = detection_cote(c1x, c1y, c3x, c3y, c4x, c4y, c2x, c2y);
    d->associe = false;
  }
  send_aruco_frame(id, type, x, y);
  draw_line(rgb_buf, w, c1x, c1y, c3x, c3y);
  draw_line(rgb_buf, w, c3x, c3y, c4x, c4y);
  draw_line(rgb_buf, w, c4x, c4y, c2x, c2y);
  draw_line(rgb_buf, w, c2x, c2y, c1x, c1y);
  dessiner_repere(rgb_buf, w, h, c1x, c1y, c3x, c3y, c4x, c4y, c2x, c2y, orientation);
  if (type == 'P') {
    int mm = profondeur_mm(c1x, c1y, c3x, c3y, c4x, c4y, c2x, c2y);
    int haut = c1y;
    if (c3y < haut) haut = c3y;
    if (c4y < haut) haut = c4y;
    if (c2y < haut) haut = c2y;
    int gauche = c1x;
    if (c3x < gauche) gauche = c3x;
    if (c4x < gauche) gauche = c4x;
    if (c2x < gauche) gauche = c2x;
    int ty = haut - 8;
    if (ty < 0) ty = haut + 2;
    tracer_cm(rgb_buf, w, h, gauche, ty, mm);
  }
}

static void effacer_composante(uint8_t* buf, int w, int h, int x0, int y0) {
  static uint16_t pileX[8192];
  static uint16_t pileY[8192];
  if (buf[y0 * w + x0] != 0) return;
  int sp = 0;
  buf[y0 * w + x0] = 1;
  pileX[sp] = (uint16_t)x0;
  pileY[sp] = (uint16_t)y0;
  sp++;
  const int dx[4] = { 1, -1, 0, 0 };
  const int dy[4] = { 0, 0, 1, -1 };
  while (sp > 0) {
    sp--;
    int x = pileX[sp];
    int y = pileY[sp];
    for (int k = 0; k < 4; k++) {
      int nx = x + dx[k];
      int ny = y + dy[k];
      if ((unsigned)nx >= (unsigned)w || (unsigned)ny >= (unsigned)h) continue;
      if (buf[ny * w + nx] != 0) continue;
      buf[ny * w + nx] = 1;
      if (sp < 8191) {
        pileX[sp] = (uint16_t)nx;
        pileY[sp] = (uint16_t)ny;
        sp++;
      }
      // Pas de clear bbox en débordement : ça détruisait les tags sur fond noir.
    }
  }
}

// Efface le noir connecté au bord de l'image (fond), sans toucher aux îlots ArUco.
static void effacer_fond_cadre(uint8_t* buf, int w, int h) {
  for (int pass = 0; pass < 128; pass++) {
    bool progress = false;
    for (int x = 0; x < w; x++) {
      if (buf[x] == 0) {
        effacer_composante(buf, w, h, x, 0);
        progress = true;
      }
      if (buf[(h - 1) * w + x] == 0) {
        effacer_composante(buf, w, h, x, h - 1);
        progress = true;
      }
    }
    for (int y = 0; y < h; y++) {
      if (buf[y * w] == 0) {
        effacer_composante(buf, w, h, 0, y);
        progress = true;
      }
      if (buf[y * w + (w - 1)] == 0) {
        effacer_composante(buf, w, h, w - 1, y);
        progress = true;
      }
    }
    if (!progress) break;
  }
}

void process_y_band(camera_fb_t* fb, uint8_t* rgb_buf, uint16_t yS, uint16_t yE, uint16_t xS, uint16_t xE) {
  uint16_t w = fb->width;
  uint16_t h = fb->height;
  if (xE > w) xE = w;
  if (xS >= xE) return;
  for (uint16_t y = yS; y < yE; y++) {
    uint8_t* row = fb->buf + (y * w);
    for (uint16_t x = xS; x < xE; x++) {
      if (row[x] != 0) continue;
      int voisins = 0;
      bool bord = false;
      if (x == 0 || row[x - 1] != 0) bord = true;
      else voisins++;
      if (x + 1 >= w || row[x + 1] != 0) bord = true;
      else voisins++;
      if (y == 0 || fb->buf[(y - 1) * w + x] != 0) bord = true;
      else voisins++;
      if (y + 1 >= h || fb->buf[(y + 1) * w + x] != 0) bord = true;
      else voisins++;
      if (!bord || voisins < 2) {
        if (voisins < 2) row[x] = 1;
        continue;
      }
      process_and_draw_aruco(fb, rgb_buf, x, y);
    }
  }
}

void analyser_frame(camera_fb_t* fb, uint8_t* rgb_buf, int* pierres, int* inconnus, int* noirs) {
  bilan_pierres = 0;
  bilan_inconnus = 0;
  ndets = 0;
  largeur_image = (int)fb->width;
  int w = (int)fb->width;
  int h = (int)fb->height;
  if (w < 1 || h < 1 || h > HIST_Y_MAX) {
    if (pierres) *pierres = 0;
    if (inconnus) *inconnus = 0;
    if (noirs) *noirs = 0;
    return;
  }

  int64_t t_now = esp_timer_get_time();
  float dt = 0.05f;
  if (t_prev_us > 0) {
    float dus = (float)(t_now - t_prev_us);
    dt = dus / 1000000.f;
  }
  if (dt < 0.005f) dt = 0.005f;
  if (dt > 0.2f) dt = 0.2f;
  t_prev_us = t_now;

  float q = 8.f * dt;
  float r_mesure = 9.f;
  int pistes_actives = 0;
  for (int i = 0; i < PISTE_MAX; i++) {
    if (!pistes[i].actif) continue;
    pistes_actives++;
    pistes[i].vu = false;
    filtre_predire(&pistes[i].fx, dt, q);
    filtre_predire(&pistes[i].fy, dt, q);
  }

  bool full = (pistes_actives == 0) || (frames_depuis_full >= FULLSCAN_PERIOD);
  frames_depuis_full = full ? 0 : (frames_depuis_full + 1);

  binarize_and_histY(fb, rgb_buf, histY);

  if (full) {
    scanner_zone(fb, rgb_buf, 0, 0, w - 1, h - 1);
  } else {
    for (int i = 0; i < PISTE_MAX; i++) {
      if (!pistes[i].actif) continue;
      int x0, y0, x1, y1;
      piste_roi(&pistes[i], w, h, &x0, &y0, &x1, &y1);
      draw_debug_rect(rgb_buf, w, x0, y0, x1, y1);
      scanner_zone(fb, rgb_buf, x0, y0, x1, y1);
    }
  }

  for (int i = 0; i < ndets; i++) {
    Detection* d = &dets[i];
    if (d->type != 'P' || d->cote < 1.f) continue;
    int meilleur = -1;
    float meilleur_d2 = 1000000000.f;
    for (int p = 0; p < PISTE_MAX; p++) {
      if (!pistes[p].actif || pistes[p].vu) continue;
      if (!dans_gate(&pistes[p], d)) continue;
      float dx = d->cx - pistes[p].fx.x;
      float dy = d->cy - pistes[p].fy.x;
      float d2 = dx * dx + dy * dy;
      if (d2 < meilleur_d2) {
        meilleur_d2 = d2;
        meilleur = p;
      }
    }
    if (meilleur >= 0) {
      Piste* p = &pistes[meilleur];
      filtre_corriger(&p->fx, d->cx, r_mesure);
      filtre_corriger(&p->fy, d->cy, r_mesure);
      p->cote = 0.7f * p->cote + 0.3f * d->cote;
      p->id = d->id;
      p->orientation = d->orientation;
      p->vu = true;
      p->perdu = 0;
      p->age++;
      d->associe = true;
    } else {
      piste_creer(d);
      d->associe = true;
    }
  }

  for (int i = 0; i < PISTE_MAX; i++) {
    if (!pistes[i].actif) continue;
    if (pistes[i].vu) continue;
    pistes[i].perdu++;
    if (pistes[i].perdu >= KALMAN_PERDU_MAX) {
      Serial.printf("Piste-%d perdue\n", i);
      pistes[i].actif = false;
      frames_depuis_full = FULLSCAN_PERIOD;
    }
  }

  int compte_noir = 0;
  int pixels = w * h;
  for (int i = 0; i < pixels; i++) {
    if (fb->buf[i] == 0) compte_noir++;
  }
  if (pierres) *pierres = bilan_pierres;
  if (inconnus) *inconnus = bilan_inconnus;
  if (noirs) *noirs = compte_noir;
}

//-----------------------------------------------------------------------------------------------
void ledState(bool state) {
  ledcWrite(canalPWM, (state) ? 2000 : 0);
}

void save_processed_photo(uint8_t* rgb_buf, int w, int h, char* name) {
  uint8_t* jpg_buf = NULL;
  size_t jpg_buf_len = 0;
  bool converted = fmt2jpg(rgb_buf, w * h * 3, w, h, PIXFORMAT_RGB888, 80, &jpg_buf, &jpg_buf_len);
  if (converted) {
    String path = "/" + String(name) + ".jpg";
    File file = SD_MMC.open(path.c_str(), FILE_WRITE);
    if (file) {
      file.write(jpg_buf, jpg_buf_len);
      file.close();
      Serial.printf("Image AVEC DESSINS sauvegardée : %s\n", path.c_str());
    }
    free(jpg_buf);
  } else {
    Serial.println("Erreur de conversion fmt2jpg");
  }
}

void check_for_commands() {
  if (Serial.available() > 0) {
    char c = Serial.read();

    if (c == 'O') {
      ledState(true);
    } else if (c == 'C') {
      ledState(false);
    } else if (c == 'I') {
      memset(pictureName, 0, sizeof(pictureName));

      int i = 0;
      while (Serial.available() > 0) {
        char next = Serial.read();
        if (next == '\n' || next == '\r') break;
        if (i < (sizeof(pictureName) - 1)) {
          pictureName[i++] = next;
        }
      }

      if (i == 0) {
        strncpy(pictureName, "capture", sizeof(pictureName) - 1);
      }
      capture = true;
    }
  }
}

void video_handler() {
  camera_fb_t* fb = NULL;
  uint8_t* _jpg_buf = NULL;
  size_t _jpg_buf_len = 0;
  char part_buf[64];
  while (true) {
    check_for_commands();
    int64_t start_time = esp_timer_get_time();
    fb = esp_camera_fb_get();
    if (!fb) {
      break;
    }

    uint8_t* rgb_buf = (uint8_t*)malloc(fb->width * fb->height * 3);
    if (rgb_buf) {
      analyser_frame(fb, rgb_buf, NULL, NULL, NULL);
      fmt2jpg(rgb_buf, fb->width * fb->height * 3, fb->width, fb->height, PIXFORMAT_RGB888, QUALITE_JPEG, &_jpg_buf, &_jpg_buf_len);
      if (capture) {
        save_processed_photo(rgb_buf, fb->width, fb->height, pictureName);
        capture = false;
      }
      free(rgb_buf);
      free(_jpg_buf);
    }
    esp_camera_fb_return(fb);
  }
}

static esp_err_t stream_handler(httpd_req_t* req) {
  camera_fb_t* fb = NULL;
  uint8_t* _jpg_buf = NULL;
  size_t _jpg_buf_len = 0;
  char part_buf[64];

  if (httpd_resp_set_type(req, _STREAM_CONTENT_TYPE) != ESP_OK) return ESP_FAIL;

  while (true) {
    check_for_commands();
    int64_t start_time = esp_timer_get_time();
    fb = esp_camera_fb_get();
    if (!fb) break;

    uint8_t* rgb_buf = (uint8_t*)malloc(fb->width * fb->height * 3);
    if (rgb_buf) {
      analyser_frame(fb, rgb_buf, NULL, NULL, NULL);
      fmt2jpg(rgb_buf, fb->width * fb->height * 3, fb->width, fb->height, PIXFORMAT_RGB888, QUALITE_JPEG, &_jpg_buf, &_jpg_buf_len);

      if (capture) {
        save_processed_photo(rgb_buf, fb->width, fb->height, pictureName);
        capture = false;
      }

      free(rgb_buf);
    }
    esp_camera_fb_return(fb);
    if (_jpg_buf) {
      size_t hlen = snprintf(part_buf, 64, _STREAM_PART, _jpg_buf_len);
      httpd_resp_send_chunk(req, part_buf, hlen);
      httpd_resp_send_chunk(req, (const char*)_jpg_buf, _jpg_buf_len);
      httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    // Stats FPS
    //int64_t frame_time = esp_timer_get_time() - start_time;
    //Serial.printf("FPS: %.2f\n", 1000000.0f / frame_time);
  }
  return ESP_OK;
}

static esp_err_t index_handler(httpd_req_t* req) {
  String page = html_generator(seuil_noir);
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, page.c_str(), page.length());
}

static esp_err_t seuil_handler(httpd_req_t* req) {
  char query[64] = { 0 };
  char valeur[12] = { 0 };
  bool lu = false;
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
      httpd_query_key_value(query, "v", valeur, sizeof(valeur)) == ESP_OK) {
    lu = true;
  }
  if (lu) {
    int v = atoi(valeur);
    if (v < 0) v = 0;
    if (v > 255) v = 255;
    seuil_noir = v;
    Serial.printf("Seuil %d\n", seuil_noir);
  }
  char reponse[16];
  int n = snprintf(reponse, sizeof(reponse), "%d", seuil_noir);
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, reponse, n);
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 8;
  config.max_open_sockets = 4;
  config.lru_purge_enable = true;

  // Port 80 : page et réglage du seuil (ne doit pas être bloqué par le flux).
  config.server_port = 80;
  config.ctrl_port = 32768;
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL };
    httpd_uri_t seuil_uri = { .uri = "/seuil", .method = HTTP_GET, .handler = seuil_handler, .user_ctx = NULL };
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &seuil_uri);
  }

  // Port 81 : flux MJPEG seul (connexion longue).
  config.server_port = 81;
  config.ctrl_port = 32769;
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_uri_t stream_uri = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
  Serial.println("Serveur web :80  flux :81");
}

void stopCameraServer() {
  if (stream_httpd != NULL) {
    httpd_stop(stream_httpd);
    stream_httpd = NULL;
  }
  if (camera_httpd != NULL) {
    httpd_stop(camera_httpd);
    camera_httpd = NULL;
  }
  Serial.println("Serveur arrêté proprement.");
}

bool getWifiMode() {
  return wifiMode;
}

void setWifiMode(bool state) {
  wifiMode = state;
}