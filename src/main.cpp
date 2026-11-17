// MotoRaiz Speedo v0.7 - Cardputer-Adv + Cap LoRa-1262 (GNSS)
// Teclas:  R = reset viagem   B = brilho   O = rodar ecrã 180 graus
//
// Pinos GNSS (docs M5Stack Cap LoRa-1262):
//   G15 = TX do GPS -> RX do ESP32
//   G13 = RX do GPS <- TX do ESP32
//
// v0.7: GPS a 5 atualizações/s, hora de Portugal (verão/inverno automático),
//       tempo em movimento e velocidade média, números grandes mais pequenos.

#include <M5Cardputer.h>
#include <TinyGPSPlus.h>

static const int PIN_GPS_RX = 15;   // ESP32 recebe aqui
static const int PIN_GPS_TX = 13;   // ESP32 envia aqui
static const uint32_t BAUDS[] = {115200, 9600, 38400};
static const int N_BAUDS = sizeof(BAUDS) / sizeof(BAUDS[0]);

HardwareSerial GPSSerial(1);
TinyGPSPlus gps;
M5Canvas canvas(&M5Cardputer.Display);

// --- estado da comunicação com o GPS ---
int      baudIdx = 0;
uint32_t baudTimer = 0;
uint32_t lastOkCount = 0, lastOkT = 0;
bool     configurado = false;
bool     tentouBaud = false;

// --- viagem ---
float    maxKmh = 0;
double   tripM  = 0;
uint32_t movingMs = 0;
double   lastLat = 0, lastLng = 0;
bool     havePrev = false;
uint32_t lastPosT = 0;
float    calcKmh = 0;
bool     usandoCalc = false;

// --- frequência de atualização medida ---
uint16_t updCount = 0;
float    hz = 0;
uint32_t hzT = 0;

// --- ecrã ---
bool    brilhoMax = false;
uint8_t rotacao = 0;    // 0 ou 2: vertical (tecla O roda 180 graus)

// O Cap LoRa-1262 tem um expansor de I/O (PI4IOE5V6408) no I2C interno (G8/G9).
// O pino P0 liga o interruptor RF / LNA do GPS.
static uint8_t capAddr = 0;

void enableCap() {
  const uint8_t addrs[] = {0x43, 0x44};
  for (uint8_t a : addrs) {
    if (M5.In_I2C.start(a, false, 400000) && M5.In_I2C.stop()) { capAddr = a; break; }
  }
  if (!capAddr) return;
  const uint32_t f = 400000;
  uint8_t dir  = M5.In_I2C.readRegister8(capAddr, 0x03, f);
  uint8_t hiz  = M5.In_I2C.readRegister8(capAddr, 0x07, f);
  uint8_t outv = M5.In_I2C.readRegister8(capAddr, 0x05, f);
  M5.In_I2C.writeRegister8(capAddr, 0x03, dir | 0x01, f);
  M5.In_I2C.writeRegister8(capAddr, 0x05, outv | 0x01, f);
  M5.In_I2C.writeRegister8(capAddr, 0x07, hiz & ~0x01, f);
}

void startGPS() {
  GPSSerial.end();
  GPSSerial.begin(BAUDS[baudIdx], SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  baudTimer = millis();
  lastOkT = millis();
}

// Envia um comando CASIC ($PCASxx...) com o checksum calculado.
// Estas definições não ficam gravadas: ao desligar, o GPS volta ao normal.
void sendCAS(const char* body) {
  uint8_t cs = 0;
  for (const char* p = body; *p; p++) cs ^= (uint8_t)*p;
  char buf[64];
  snprintf(buf, sizeof(buf), "$%s*%02X\r\n", body, cs);
  GPSSerial.print(buf);
  GPSSerial.flush();
}

// Muda o GPS para 115200 baud (se preciso), só frases GGA + RMC, e 5 atualizações/s
void configureGPS() {
  if (configurado || gps.passedChecksum() < 5) return;
  if (BAUDS[baudIdx] != 115200 && !tentouBaud) {
    tentouBaud = true;
    sendCAS("PCAS01,5");          // 115200 baud
    delay(100);
    baudIdx = 0;
    startGPS();
    return;                        // configura o resto quando voltar a receber frases
  }
  sendCAS("PCAS03,1,0,0,0,1,0,0,0");   // só GGA e RMC
  delay(50);
  sendCAS("PCAS02,200");               // 200 ms = 5 atualizações por segundo
  configurado = true;
}

// Se deixar de chegar frases válidas durante 3 s, experimenta outro baud
void checkBaud() {
  uint32_t ok = gps.passedChecksum();
  if (ok != lastOkCount) { lastOkCount = ok; lastOkT = millis(); return; }
  if (millis() - lastOkT > 3000 && millis() - baudTimer > 3000) {
    baudIdx = (baudIdx + 1) % N_BAUDS;
    configurado = false;
    startGPS();
  }
}

void resetTrip() {
  maxKmh = 0;
  tripM = 0;
  movingMs = 0;
  havePrev = false;
}

bool bomSinal() {
  return !gps.hdop.isValid() || gps.hdop.hdop() < 3.0;
}

void processLocation() {
  if (!gps.location.isValid() || !gps.location.isUpdated()) return;
  updCount++;
  double lat = gps.location.lat(), lng = gps.location.lng();
  uint32_t now = millis();

  if (!havePrev) {
    lastLat = lat; lastLng = lng; lastPosT = now; havePrev = true;
    return;
  }
  uint32_t dt = now - lastPosT;
  if (dt < 1000) return;                       // distância medida de 1 em 1 s

  double d = TinyGPSPlus::distanceBetween(lastLat, lastLng, lat, lng);
  float v = (float)(d / (dt / 1000.0) * 3.6);
  if (dt > 5000) calcKmh = 0;
  else if (v < 250) calcKmh = 0.6f * v + 0.4f * calcKmh;

  if (bomSinal() && d > 2.0 && d < 200.0 && v > 3.0) tripM += d;
  lastLat = lat; lastLng = lng; lastPosT = now;
}

float currentKmh(bool fix) {
  if (!fix) return 0;
  float kmh;
  if (gps.speed.isValid() && gps.speed.age() < 3000) {
    kmh = gps.speed.kmph();
    usandoCalc = false;
  } else {
    kmh = calcKmh;
    usandoCalc = true;
  }
  if (kmh < 2.0f) kmh = 0;
  if (bomSinal() && kmh > maxKmh) maxKmh = kmh;
  return kmh;
}

// ---------- Hora de Portugal continental ----------
// Inverno: UTC+0. Verão: UTC+1, do último domingo de março (01:00 UTC)
// ao último domingo de outubro (01:00 UTC).
int diaSemana(int y, int m, int d) {           // 0 = domingo
  static const int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  if (m < 3) y--;
  return (y + y / 4 - y / 100 + y / 400 + t[m - 1] + d) % 7;
}

bool horaVerao(int y, int m, int d, int h) {
  if (m < 3 || m > 10) return false;
  if (m > 3 && m < 10) return true;
  int ultimoDomingo = 31 - diaSemana(y, m, 31);
  if (m == 3) return d > ultimoDomingo || (d == ultimoDomingo && h >= 1);
  return d < ultimoDomingo || (d == ultimoDomingo && h < 1);
}

String horaLocal() {
  if (!gps.time.isValid() || !gps.date.isValid() || gps.date.year() < 2020) return "";
  int h = gps.time.hour();
  if (horaVerao(gps.date.year(), gps.date.month(), gps.date.day(), h)) h = (h + 1) % 24;
  char t[6];
  snprintf(t, sizeof(t), "%02d:%02d", h, gps.time.minute());
  return String(t);
}

String duracao(uint32_t ms) {
  uint32_t s = ms / 1000;
  char t[10];
  snprintf(t, sizeof(t), "%u:%02u:%02u", (unsigned)(s / 3600), (unsigned)(s / 60 % 60), (unsigned)(s % 60));
  return String(t);
}

// ---------- Ecrã ----------
void draw(float kmh, bool fix) {
  canvas.fillSprite(TFT_BLACK);
  const int W = canvas.width(), H = canvas.height(), CX = W / 2;

  // Barra de estado
  canvas.setFont(&fonts::Font2);
  canvas.setTextDatum(top_left);
  canvas.setTextColor(fix ? TFT_GREEN : TFT_ORANGE, TFT_BLACK);
  canvas.drawString(fix ? "GPS OK" : "Procura...", 3, 2);

  canvas.setTextDatum(top_right);
  canvas.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
  canvas.drawString(String("Sat ") + sats, W - 3, 2);

  // Hora local (Portugal)
  String hl = horaLocal();
  if (hl.length()) {
    canvas.setTextDatum(top_center);
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    canvas.drawString(hl, CX, 20);
  }

  // Diagnóstico (pequeno)
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(TFT_DARKGREY, TFT_BLACK);
  canvas.setTextDatum(top_center);
  if (!fix) {
    canvas.drawString(String("RX ") + gps.charsProcessed() + " OK " + gps.passedChecksum(), CX, 40);
    canvas.drawString(String("@") + BAUDS[baudIdx] + " IO 0x" + String(capAddr, HEX), CX, 50);
  } else {
    canvas.drawString(String(usandoCalc ? "calc " : "GPS ") + String(hz, 0) + "Hz", CX, 40);
  }

  // Velocidade (grande, ao centro)
  int v = (int)(kmh + 0.5f);
  canvas.setFont(&fonts::Font7);
  canvas.setTextSize(v >= 100 ? 1.1f : 1.45f);
  canvas.setTextDatum(middle_center);
  canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  canvas.drawString(fix ? String(v) : String("--"), CX, H / 2 - 12);
  canvas.setTextSize(1);

  canvas.setFont(&fonts::Font2);
  canvas.setTextDatum(top_center);
  canvas.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  canvas.drawString("km/h", CX, H / 2 + 26);

  // Rodapé: máxima / média e distância / tempo em movimento
  float medKmh = movingMs > 0 ? (float)(tripM / 1000.0) / (movingMs / 3600000.0f) : 0;
  canvas.drawFastHLine(8, H - 64, W - 16, TFT_DARKGREY);
  canvas.setTextColor(TFT_CYAN, TFT_BLACK);
  canvas.setTextDatum(top_left);
  canvas.drawString(String("Max ") + (int)(maxKmh + 0.5f), 4, H - 58);
  canvas.drawString(String(tripM / 1000.0, 2) + "km", 4, H - 36);
  canvas.setTextDatum(top_right);
  canvas.drawString(String("Med ") + (int)(medKmh + 0.5f), W - 4, H - 58);
  canvas.drawString(duracao(movingMs), W - 4, H - 36);

  canvas.pushSprite(0, 0);
}

void setup() {
  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);
  M5Cardputer.Display.setRotation(rotacao);
  M5Cardputer.Display.fillScreen(TFT_BLACK);
  M5Cardputer.Display.setBrightness(120);
  canvas.createSprite(M5Cardputer.Display.width(), M5Cardputer.Display.height());
  Serial.begin(115200);
  enableCap();
  startGPS();
}

void loop() {
  M5Cardputer.update();

  // Teclado
  if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) {
    auto st = M5Cardputer.Keyboard.keysState();
    for (auto c : st.word) {
      if (c == 'r' || c == 'R') resetTrip();
      if (c == 'o' || c == 'O') {
        rotacao = (rotacao == 0) ? 2 : 0;
        M5Cardputer.Display.setRotation(rotacao);
        M5Cardputer.Display.fillScreen(TFT_BLACK);
      }
      if (c == 'b' || c == 'B') {
        brilhoMax = !brilhoMax;
        M5Cardputer.Display.setBrightness(brilhoMax ? 255 : 120);
      }
    }
  }

  // Ler GNSS
  while (GPSSerial.available()) gps.encode(GPSSerial.read());
  checkBaud();
  configureGPS();

  bool fix = gps.location.isValid() && gps.location.age() < 3000;
  processLocation();
  float kmh = currentKmh(fix);

  // Tempo em movimento
  static uint32_t lastLoop = millis();
  uint32_t now = millis();
  if (fix && kmh > 3.0f) movingMs += now - lastLoop;
  lastLoop = now;

  // Frequência medida (atualizações de posição por segundo)
  if (now - hzT >= 2000) {
    hz = updCount * 1000.0f / (now - hzT);
    updCount = 0;
    hzT = now;
  }

  static uint32_t lastDraw = 0;
  if (now - lastDraw > 100) {
    lastDraw = now;
    draw(kmh, fix);
  }
}
