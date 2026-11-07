// PHẦN 1: KHAI BÁO CHÂN PHẦN CỨNG
// =============================================================================
// --- Động cơ DC & Mạch cầu TB6612 ---
const int AIN1 = 26, AIN2 = 27, PWMA = 14;     // Động cơ Trái
const int BIN1 = 25, BIN2 = 33, PWMB = 32;     // Động cơ Phải

// --- Cảm biến Dò Line TCRT5000 (5 mắt S1 -> S5 từ Trái qua Phải) ---
const int LINE_PIN[5] = {34, 35, 36, 39, 18};

// --- Cảm biến Siêu âm HC-SR04 ---
const int TRIG_PIN = 5;                        // Chân phát TRIG
const int ECHO_PIN = 17;                       // Chân nhận ECHO


// =============================================================================
// PHẦN 2: THÔNG SỐ ĐIỀU KHIỂN ĐỘ CHÍNH XÁC CAO (HIGH PRECISION TUNING)
// =============================================================================
// --- Cấu hình Cảm biến Siêu âm ---
const float OBSTACLE_DIST_CM  = 10.0f;         // Dừng khẩn cấp ở khoảng cách 10cm
const float OBSTACLE_CLEAR_CM = 13.0f;         // Chạy tiếp khi vật cản ra xa 13cm
const unsigned long US_READ_INTERVAL = 50;     // Chu kỳ đọc siêu âm (50ms)

// --- Cấu hình Động cơ PWM ---
const bool INVERT_LEFT  = false;
const bool INVERT_RIGHT = false;
const int  PWM_MAX      = 220;                 
int        basePwm      = 45;                  // TỐC ĐỘ CƠ SỞ CHẬM (PWM = 45) ĐỂ BÁM LINE CHÍNH XÁC

// --- Bộ điều khiển PID Bám Line ---
const unsigned long CTRL_MS = 20;              // Chu kỳ tính toán (20ms)
float LKP = 120.0f;                            // Kp vi sai bẻ lái
float LKI = 0.6f;                              // Ki triệt tiêu sai số tích tụ
float LKD = 16.0f;                             // Kd chống lắc đuôi xe
const float LINT_LIM = 35.0f;                  

// --- Bộ Mờ (Fuzzy Logic) - Cấu hình giảm tốc theo vị trí LED ---
bool  useFuzzy = true;                         
float DE_MAX = 6.0f;

// SPD_SCALE: 
// Index 0 (Lệch sâu LED 1/5): Chạy 25% basePwm (Rất chậm để quay đầu)
// Index 1 (Lệch vừa LED 2/4): Chạy 55% basePwm (Chậm vừa để căn lại vạch)
// Index 2 (Vạch giữa LED 3) : Chạy 100% basePwm (Tốc độ chuẩn)
const float KP_SCALE[3]  = {0.80f, 1.00f, 1.50f};
const float KD_SCALE[3]  = {0.50f, 1.00f, 1.80f};
const float SPD_SCALE[3] = {0.25f, 0.55f, 1.00f};
const float SPD_SMOOTH   = 0.20f;              

// Bảng luật Fuzzy điều khiển giảm tốc độ theo mức độ lệch
const uint8_t RULE_KP[5][5] = {
  {2, 2, 2, 2, 1}, {2, 2, 1, 1, 0}, {1, 0, 0, 0, 1}, {0, 1, 1, 2, 2}, {1, 2, 2, 2, 2}
};
const uint8_t RULE_KD[5][5] = {
  {0, 0, 0, 1, 1}, {0, 0, 1, 2, 2}, {2, 1, 0, 1, 2}, {2, 2, 1, 0, 0}, {1, 1, 0, 0, 0}
};
const uint8_t RULE_SPD[5][5] = {
  {0, 0, 0, 1, 1}, 
  {0, 1, 1, 2, 2}, 
  {1, 2, 2, 2, 1}, // Hàng giữa: LED3 ở tâm -> SPD cao nhất
  {2, 2, 1, 1, 0}, 
  {1, 1, 0, 0, 0}
};

// --- Cấu hình Cảm biến TCRT5000 ---
const bool  LINE_DIGITAL = true;               
const bool  LINE_IS_HIGH = false;              // Vạch đen = 0, Phông trắng = 1
const float POS_FILTER = 0.85f;                
const float D_FILTER = 0.25f;
const float LINE_W[5] = {-2000, -1000, 0, 1000, 2000};
const float LINE_DETECT_MIN = 150.0f;

const unsigned long AUTO_STOP_MS = 120000;
const int AUTOSTART_PWM = 45;                  // Tự động chạy với PWM = 45 sau 3s


// =============================================================================
// PHẦN 3: STRUCT & BIẾN TRẠNG THÁI
// =============================================================================
const int MOTOR_FREQ = 1000, MOTOR_RES = 8;
const int CH_A = 0, CH_B = 1;

enum RobotMode { M_STOP, M_LINE, M_OBSTACLE_HOLD };
RobotMode mode = M_STOP;

unsigned long lastControlMs = 0, runStartMs = 0, lastUsReadMs = 0;
int printDiv = 0;

float linePrevE = 0, lineInteg = 0, dFilt = 0, lastE = 0;
int lastSide = 1;

float spdFilt = 1.0f;
float currentDistanceCm = 999.0f;
int currentPwmL = 0, currentPwmR = 0;


// =============================================================================
// PHẦN 4: MODULE CẢM BIẾN SIÊU ÂM HC-SR04 (ĐỌC 10CM)
// =============================================================================
void setupUltrasonic() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
}

float readUltrasonicCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 12000); // Timeout 12ms
  if (duration == 0) return 999.0f;
  return (float)duration * 0.0343f / 2.0f;
}

void checkObstacleTask() {
  unsigned long now = millis();
  if (now - lastUsReadMs < US_READ_INTERVAL) return;
  lastUsReadMs = now;

  float dist = readUltrasonicCm();
  if (dist > 0.0f) currentDistanceCm = dist;

  // Phát hiện vật cản <= 10cm -> Dừng khẩn cấp
  if (mode == M_LINE && currentDistanceCm <= OBSTACLE_DIST_CM) {
    mode = M_OBSTACLE_HOLD;
    drive(0, 0);
    Serial.printf("![DỪNG KHẨN CẤP] Vat can o %.1f cm (<= 10cm)!\n", currentDistanceCm);
  } 
  else if (mode == M_OBSTACLE_HOLD && currentDistanceCm >= OBSTACLE_CLEAR_CM) {
    mode = M_LINE;
    Serial.printf("[THÔNG BÁO] Vat can da di chuyen (%.1f cm) -> Tiep tuc bam line!\n", currentDistanceCm);
  }
}
// =============================================================================
// PHẦN 5: MODULE ĐIỀU KHIỂN ĐỘNG CƠ (PWM)
// =============================================================================
void pwmSetup(int pin, int ch, int freq, int res) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, freq, res);
#else
  ledcSetup(ch, freq, res);
  ledcAttachPin(pin, ch);
#endif
}

void pwmWrite(int pin, int ch, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(ch, duty);
#endif
}

void setMotor(int in1, int in2, int pwmPin, int ch, int speed, bool invert) {
  if (invert) speed = -speed;
  speed = constrain(speed, -PWM_MAX, PWM_MAX);

  if (speed > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);  digitalWrite(in2, LOW);
  }
  pwmWrite(pwmPin, ch, (uint32_t)abs(speed));
}

void drive(int leftPwm, int rightPwm) {
  currentPwmL = leftPwm;
  currentPwmR = rightPwm;
  setMotor(AIN1, AIN2, PWMA, CH_A, leftPwm,  INVERT_LEFT);
  setMotor(BIN1, BIN2, PWMB, CH_B, rightPwm, INVERT_RIGHT);
}


// =============================================================================
// PHẦN 6: MODULE CẢM BIẾN LINE & THUẬT TOÁN FUZZY-PID
// =============================================================================
int calMin[5] = {0}, calMax[5] = {4095, 4095, 4095, 4095, 4095};
float norm[5] = {0};
float posFilt = 0;
bool posFiltValid = false;

bool readLine(float &pos, float &sum) {
  float num = 0, den = 0;
  for (int i = 0; i < 5; i++) {
    float v;
    if (LINE_DIGITAL) {
      v = digitalRead(LINE_PIN[i]) ? 1000.0f : 0.0f;
    } else {
      int raw = analogRead(LINE_PIN[i]);
      int range = calMax[i] - calMin[i];
      if (range < 50) range = 50;
      v = (raw - calMin[i]) * 1000.0f / range;
      v = constrain(v, 0.0f, 1000.0f);
    }
    if (!LINE_IS_HIGH) v = 1000.0f - v;
    norm[i] = v;
    num += v * LINE_W[i];
    den += v;
  }
  sum = den;
  if (den < LINE_DETECT_MIN) {
    posFiltValid = false;
    return false;
  }
  float p = num / den;
  if (LINE_DIGITAL) {
    if (!posFiltValid) { posFilt = p; posFiltValid = true; }
    else posFilt += POS_FILTER * (p - posFilt);
    p = posFilt;
  }
  pos = p;
  return true;
}

void memberships(float x, float mu[5]) {
  x = constrain(x, -1.0f, 1.0f);
  float t = (x + 1.0f) * 2.0f;
  int i = (int)t;
  if (i > 3) i = 3;
  float f = t - i;
  for (int k = 0; k < 5; k++) mu[k] = 0;
  mu[i] = 1.0f - f; mu[i + 1] = f;
}

void fuzzyInfer(float e, float de, float &kp, float &kd, float &spd) {
  float me[5], md[5];
  memberships(e, me);
  memberships(de / DE_MAX, md);

  float sw = 0, skp = 0, skd = 0, ssp = 0;
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 5; j++) {
      float w = me[i] * md[j];
      if (w <= 0) continue;
      sw  += w;
      skp += w * KP_SCALE[RULE_KP[i][j]];
skd += w * KD_SCALE[RULE_KD[i][j]];
      ssp += w * SPD_SCALE[RULE_SPD[i][j]];
    }
  }
  kp  = LKP * skp / sw;
  kd  = LKD * skd / sw;
  spd = ssp / sw;
}

void stopAll() {
  mode = M_STOP;
  drive(0, 0);
  Serial.println("Da dung xe.");
}

void startLine(int pwm) {
  basePwm = constrain(pwm, 30, PWM_MAX);
  linePrevE = 0; lineInteg = 0; dFilt = 0; lastE = 0;
  lastSide = 1; spdFilt = 1.0f;
  runStartMs = millis(); mode = M_LINE;
  Serial.printf("Bat dau bam line [%s] voi toc do PWM %d\n", useFuzzy ? "Fuzzy-PID" : "PID Thuong", basePwm);
}

void lineControl(float dt) {
  float pos = 0, sum = 0;
  bool found = readLine(pos, sum);

  if (found) {
    float e = pos / 2000.0f; // e thuộc [-1.0, 1.0]
    if (fabsf(e) > 0.15f) lastSide = (e > 0) ? 1 : -1;

    float d = (e - linePrevE) / dt;
    linePrevE = e;
    dFilt += D_FILTER * (d - dFilt);
    lineInteg = constrain(lineInteg + LKI * e * dt, -LINT_LIM, LINT_LIM);

    float kpEff = LKP, kdEff = LKD, spdFac = 1.0f;
    if (useFuzzy) {
      float spdTarget = 1.0f;
      fuzzyInfer(e, dFilt, kpEff, kdEff, spdTarget);
      spdFilt += SPD_SMOOTH * (spdTarget - spdFilt);
      spdFac = spdFilt; // spdFac tự động giảm từ 1.0 -> 0.55 -> 0.25 khi sai số e tăng
    }

    float u = kpEff * e + lineInteg + kdEff * dFilt;
    float vBase = basePwm * spdFac; // Tốc độ chạy thẳng bị hạ thấp khi vào cua

    // Phối hợp bẻ lái vi sai
    int pwmL = (int)(vBase + u);
    int pwmR = (int)(vBase - u);

    drive(pwmL, pwmR);
    lastE = e;
  } else {
    // Mất vạch hoàn toàn: Xoay tại chỗ chậm (50 PWM) để tìm lại vạch
    int sweepPwm = 50;
    drive(-lastSide * sweepPwm, lastSide * sweepPwm);
  }
}

void controlStep() {
  float dt = CTRL_MS / 1000.0f;

  if (mode == M_OBSTACLE_HOLD || mode == M_STOP) {
    drive(0, 0);
    return;
  }

  if (mode == M_LINE) {
    lineControl(dt);
  }

  if ((++printDiv & 1) == 0) {
    Serial.printf("Dist:%.1fcm | e:%.2f, PwmL:%d, PwmR:%d\n",
                  currentDistanceCm, lastE, currentPwmL, currentPwmR);
  }
}

void showSensors() {
  Serial.println("Kiem tra 5 mat cam bien (5s):");
  unsigned long t0 = millis();
  while (millis() - t0 < 5000) {
    float pos = 0, sum = 0;
    bool found = readLine(pos, sum);
    Serial.printf("%4.0f %4.0f %4.0f %4.0f %4.0f | Vi tri=%s%.0f\n",
                  norm[0], norm[1], norm[2], norm[3], norm[4],
                  found ? "" : "(Mat vach) ", found ? pos : 0.0f);
    delay(100);
  }
}

Preferences prefs;
void saveParams() {
  prefs.begin("xeline", false);
  prefs.putFloat("lkp", LKP); prefs.putFloat("lki", LKI); prefs.putFloat("lkd", LKD);
  prefs.putInt("basePwm", basePwm); prefs.putFloat("dmax", DE_MAX);
  prefs.putBool("fz", useFuzzy);
  prefs.end();
}

void loadParams() {
  prefs.begin("xeline", true);
  LKP = prefs.getFloat("lkp", LKP); LKI = prefs.getFloat("lki", LKI); LKD = prefs.getFloat("lkd", LKD);
basePwm = prefs.getInt("basePwm", basePwm); DE_MAX = prefs.getFloat("dmax", DE_MAX);
  useFuzzy = prefs.getBool("fz", useFuzzy);
  prefs.end();
}

void printHelp() {
  Serial.println("\n--- DANH SACH LENH DIEU KHIEN (HIGH PRECISION 10CM) ---");
  Serial.println("go <pwm> : Chay bam line voi toc do PWM (vd: go 45)");
  Serial.println("x        : Dung xe ngap lap tuc");
  Serial.println("sens     : Xem trang thai 5 mat cam bien");
  Serial.println("fz <0|1> : 0 = PID thuong, 1 = Fuzzy-PID");
  Serial.println("lk <kp> <ki> <kd> : Thay doi PID line (vd: lk 120 0.6 16)");
  Serial.println("save     : Luu thong so vao bo nho Flash");
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  char word[16] = {0};
  float a = 0, b = 0, c = 0;
  int n = sscanf(line.c_str(), "%15s %f %f %f", word, &a, &b, &c);
  String cmd = String(word);

  if (cmd == "sens") { stopAll(); showSensors(); }
  else if (cmd == "go" && n >= 2) { startLine((int)a); }
  else if (cmd == "x") { stopAll(); }
  else if (cmd == "fz" && n >= 2) { useFuzzy = (a != 0); Serial.printf("Che do: %s\n", useFuzzy ? "Fuzzy-PID" : "PID Thuong"); }
  else if (cmd == "lk" && n >= 4) { LKP = a; LKI = b; LKD = c; Serial.printf("PID line: Kp=%.2f Ki=%.2f Kd=%.2f\n", LKP, LKI, LKD); }
  else if (cmd == "save") { saveParams(); Serial.println("Da luu thong so vao Flash."); }
  else { printHelp(); }
}


// =============================================================================
// PHẦN 7: KHỞI TẠO (SETUP) & VÒNG LẶP ĐIỀU KHIỂN CHÍNH (LOOP)
// =============================================================================
void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  for (int i = 0; i < 5; i++) pinMode(LINE_PIN[i], INPUT);

  setupUltrasonic();

  pwmSetup(PWMA, CH_A, MOTOR_FREQ, MOTOR_RES);
  pwmSetup(PWMB, CH_B, MOTOR_FREQ, MOTOR_RES);

  drive(0, 0);

  loadParams();
  printHelp();

  if (AUTOSTART_PWM > 0) {
    Serial.printf("\nXe tu dong chay sau 3 giay (Toc do PWM %d)...\n", AUTOSTART_PWM);
    delay(3000);
    startLine(AUTOSTART_PWM);
  }
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleCommand(line);
  }

  // Task 1: Đọc cảm biến siêu âm phát hiện vật cản 10cm
  checkObstacleTask();

  // Task 2: Điều khiển bám line Fuzzy-PID độ chính xác cao (20ms/lần)
  unsigned long now = millis();
  if (now - lastControlMs >= CTRL_MS) {
    lastControlMs = now;
    controlStep();
  }

  // Task 3: Giới hạn thời gian 120s
  if (mode != M_STOP && AUTO_STOP_MS > 0 && millis() - runStartMs > AUTO_STOP_MS) {
    Serial.println("Hieu luc chay 120s ket thuc, tu dong dung.");
    stopAll();
  }
}
