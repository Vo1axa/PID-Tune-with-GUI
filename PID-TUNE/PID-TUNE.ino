// TUNING PID kecepatan motor. ESP8266 + BTS7960 + encoder, tanpa RS485.
// Serial Monitor 115200, akhiran baris "Newline". Angkat roda dari lantai sebelum mulai.
//
// Perintah:
//   auto [rpm]   cari PID otomatis (metode relay). Default 80 rpm.
//   kp 3 / ki 8 / kd 0   ubah gain
//   sp 100       set target rpm untuk uji
//   go           uji PID 8 detik, lalu cetak overshoot, rise time, dan error akhir
//   stop         berhenti
//   show         tampilkan gain
//
// Buka Serial Plotter untuk melihat grafik sp, rpm, pwm.
// Bentuk PID sama dengan program utama (integral dibatasi +-150), jadi gain bisa disalin langsung.
// Hanya putaran maju. Gain hasil auto-tune adalah titik awal, rapikan dengan go dan kp/ki/kd.
#include <Arduino.h>

const int RPWM = 5, LPWM = 4, ENC_A = 12, ENC_B = 14;
const float COUNTS_PER_REV = 1320.0f;  // sama dengan program utama
const uint32_t DT = 20;                // ms
const uint32_t RUN_MS = 8000;

const float HYST = 5.0f;     // rpm, lebih besar dari noise encoder (1 hitungan ~2,3 rpm)
const int CYCLES_SKIP = 2, CYCLES_USE = 5;

volatile long enc = 0;
void IRAM_ATTR isrA() {
  if (digitalRead(ENC_A) == digitalRead(ENC_B)) enc++;
  else enc--; 
}

enum State { IDLE, RUN, RAMP, RELAY };
State state = IDLE;

float Kp = 3.0f, Ki = 8.0f, Kd = 0.0f;
float sp = 100.0f;

float rpm = 0, rpmF = 0, integ = 0, lastErr = 0;
int pwmOut = 0;
uint32_t t0 = 0;

// metrik uji
float maxRpm = 0;
uint32_t t90 = 0;
float tailSum = 0;
int tailN = 0;

// auto-tune
float atTarget = 80, bias = 0, amp = 0, cycMax, cycMin;
bool relayHigh = false;
uint32_t lastRise = 0, rampTick = 0;
int cycles = 0, used = 0;
float sumAmp = 0, sumPer = 0;

void setMotor(int pwm) {
  pwm = constrain(pwm, 0, 1023);
  pwmOut = pwm;
  analogWrite(LPWM, 0);
  analogWrite(RPWM, pwm);
}

void stopAll(const char* why) {
  state = IDLE;
  setMotor(0);
  Serial.printf("# %s\n", why);
}

void show() {
  Serial.printf("# Kp=%.3f Ki=%.3f Kd=%.3f sp=%.0f\n", Kp, Ki, Kd, sp);
}

void startRun() {
  integ = 0;
  lastErr = 0;
  maxRpm = 0;
  t90 = 0;
  tailSum = 0;
  tailN = 0;
  t0 = millis();
  state = RUN;
  Serial.println("# uji PID mulai");
}

void startAuto(float target) {
  atTarget = target;
  pwmOut = 0;
  rampTick = millis();
  t0 = millis();
  cycles = 0;
  used = 0;
  sumAmp = 0;
  sumPer = 0;
  lastRise = 0;
  state = RAMP;
  Serial.printf("# auto-tune: cari PWM untuk %.0f rpm\n", target);
}

void finishAuto() {
  float a = sumAmp / used, Tu = sumPer / used;  // Tu dalam detik
  float d = amp;
  float Ku = 4.0f * d / (PI * a);
  float zKp = 0.6f * Ku, zKi = zKp / (Tu / 2.0f), zKd = zKp * Tu / 8.0f;
  float sKp = 0.2f * Ku, sKi = sKp / (Tu / 2.0f), sKd = sKp * Tu / 3.0f;
  Serial.printf("# Ku=%.3f Tu=%.3f s (amplitudo rpm %.1f, %d siklus)\n", Ku, Tu, a, used);
  Serial.printf("# Ziegler-Nichols klasik: Kp=%.3f Ki=%.3f Kd=%.4f\n", zKp, zKi, zKd);
  Serial.printf("# Overshoot kecil:        Kp=%.3f Ki=%.3f Kd=%.4f (dipakai sekarang)\n", sKp, sKi, sKd);
  Kp = sKp;
  Ki = sKi;
  Kd = sKd;
  sp = atTarget;
  stopAll("auto-tune selesai. Kirim go untuk menguji hasilnya");
}

void handleLine(char* line) {
  char* cmd = strtok(line, " \r\n");
  if (!cmd) return;
  char* arg = strtok(nullptr, " \r\n");
  float v = arg ? atof(arg) : NAN;

  if (!strcmp(cmd, "kp") && arg) Kp = v;
  else if (!strcmp(cmd, "ki") && arg) Ki = v;
  else if (!strcmp(cmd, "kd") && arg) Kd = v;
  else if (!strcmp(cmd, "sp") && arg) sp = constrain(v, 0, 300);
  else if (!strcmp(cmd, "go")) { startRun(); return; }
  else if (!strcmp(cmd, "auto")) { startAuto(arg ? constrain(v, 20, 300) : 80); return; }
  else if (!strcmp(cmd, "stop")) { stopAll("stop"); return; }
  else if (strcmp(cmd, "show")) Serial.println("# perintah tidak dikenal");
  show();
}

void tick() {
  noInterrupts();
  long e = enc;
  interrupts();
  static long lastEnc = 0;
  rpm = (e - lastEnc) / COUNTS_PER_REV * 60.0f * (1000.0f / DT);
  lastEnc = e;
  rpmF = 0.7f * rpmF + 0.3f * rpm;

  uint32_t now = millis();

  if (state == RUN) {
    float err = sp - rpm;
    integ = constrain(integ + err * DT / 1000.0f, -150.0f, 150.0f);
    float out = Kp * err + Ki * integ + Kd * (err - lastErr) / (DT / 1000.0f);
    lastErr = err;
    setMotor((int)out);

    uint32_t t = now - t0;
    if (rpm > maxRpm) maxRpm = rpm;
    if (!t90 && rpm >= 0.9f * sp) t90 = t;
    if (t > RUN_MS - 1000) { tailSum += rpm; tailN++; }
    if (t >= RUN_MS) {
      float ss = tailN ? tailSum / tailN : 0;
      Serial.printf("# overshoot=%.1f%% rise90=%u ms error_akhir=%.1f rpm\n",
                    sp > 0 ? (maxRpm - sp) / sp * 100.0f : 0, (unsigned)t90, sp - ss);
      stopAll("uji selesai");
    }
  }

  else if (state == RAMP) {
    if (now - rampTick >= 100) {  // naikkan PWM pelan sampai target tercapai
      rampTick = now;
      setMotor(pwmOut + 5);
    }
    if (rpmF >= atTarget) {
      bias = pwmOut;
      amp = constrain(bias * 0.5f, 30.0f, min(bias, 1023.0f - bias));
      relayHigh = true;
      cycMax = -1e9;
      cycMin = 1e9;
      t0 = now;
      Serial.printf("# bias PWM=%.0f, amplitudo relay=%.0f\n", bias, amp);
      state = RELAY;
    } else if (pwmOut >= 900) {
      stopAll("target rpm tidak tercapai, turunkan nilai rpm atau cek motor");
    }
  }

  else if (state == RELAY) {
    cycMax = max(cycMax, rpm);
    cycMin = min(cycMin, rpm);

    if (relayHigh && rpm > atTarget + HYST) relayHigh = false;
    else if (!relayHigh && rpm < atTarget - HYST) {
      relayHigh = true;  // awal siklus baru
      if (lastRise) {
        cycles++;
        if (cycles > CYCLES_SKIP) {
          sumAmp += (cycMax - cycMin) / 2.0f;
          sumPer += (now - lastRise) / 1000.0f;
          used++;
        }
      }
      lastRise = now;
      cycMax = -1e9;
      cycMin = 1e9;
    }
    setMotor((int)(bias + (relayHigh ? amp : -amp)));

    if (used >= CYCLES_USE) finishAuto();
    else if (now - t0 > 40000) stopAll("auto-tune gagal: tidak ada osilasi stabil dalam 40 detik");
  }

  if (state != IDLE || pwmOut != 0)
    Serial.printf("sp:%.0f,rpm:%.1f,pwm:%d\n", state == RELAY || state == RAMP ? atTarget : sp, rpm, pwmOut);
}

void setup() {
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  digitalWrite(RPWM, LOW);
  digitalWrite(LPWM, LOW);

  Serial.begin(115200);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  analogWriteRange(1023);
  analogWriteFreq(10000);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrA, CHANGE);
  Serial.println("# Siap. Ketik auto, atau kp/ki/kd/sp lalu go");
  show();
}

void loop() {
  static char line[40];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      line[n] = 0;
      n = 0;
      handleLine(line);
    } else if (n < sizeof(line) - 1) line[n++] = c;
  }

  static uint32_t last = 0;
  if (millis() - last >= DT) {
    last += DT;
    tick();
  }
}