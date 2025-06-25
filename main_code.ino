#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Arduino_JSON.h>
#include <ESP32Servo.h>

// --- SERVO --
#define SERVO_PIN 12 
Servo miServo;

// --- WiFi ---
const char* ssid = "Note 12s";
const char* password = "micknay29";

// --- WebSocket ---
WebSocketsServer webSocket = WebSocketsServer(81);
uint8_t client_num = 0;
bool cliente_conectado = false;
bool esperando_datos = false;
bool homing_realizado = false;
bool ya_enviado_1 = false;
unsigned long t_inicio_espera = 0;

// --- Parámetros físicos ---
#define MICROSTEPS_PER_REV 3200.0f
#define DIAMETRO_POLEA_MM 12.7f
#define M_PI 3.14159265358979323846f

// --- Límites de la máquina ---
#define X_MIN_MM 0.0f
#define X_MAX_MM 210.0f
#define Y_MIN_MM 0.0f
#define Y_MAX_MM 260.0f

// === Pines Motor A (motor 2) ===
#define M2_PUL_PIN 23
#define M2_DIR_PIN 22
#define M2_ENA_PIN 21

// === Pines Motor B (motor 1) ===
#define M1_PUL_PIN 19
#define M1_DIR_PIN 18
#define M1_ENA_PIN 5

// === Botones homing ===
#define HOMING_X_BUTTON_PIN 34
#define HOMING_Y_BUTTON_PIN 35

// == PINES RELES
#define rele1 4
#define rele2 16

// === Posición lógica ===
float x_pos_mm = 0.0f;
float y_pos_mm = 0.0f;

void delayd(unsigned long ms) {
  unsigned long inicio = millis();
  while (millis() - inicio < ms) {
    webSocket.loop();  // Mantiene vivo el WebSocket
    delay(1);
  }
}
// === SERVO ===
void moveServoSlowly(int startAngle, int endAngle, int stepDelay) {
  int step = (endAngle > startAngle) ? 1 : -1;

  for (int angle = startAngle; angle != endAngle; angle += step) {
    miServo.write(angle);
    delay(stepDelay);
  }

  miServo.write(endAngle); // Asegura que termine en el ángulo exacto
}
// === Setup de pines ===
void setup_motor_pins() {
  pinMode(M1_PUL_PIN, OUTPUT);
  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M1_ENA_PIN, OUTPUT);
  digitalWrite(M1_ENA_PIN, LOW);

  pinMode(M2_PUL_PIN, OUTPUT);
  pinMode(M2_DIR_PIN, OUTPUT);
  pinMode(M2_ENA_PIN, OUTPUT);
  digitalWrite(M2_ENA_PIN, LOW);

  pinMode(HOMING_X_BUTTON_PIN, INPUT);
  pinMode(HOMING_Y_BUTTON_PIN, INPUT);

  pinMode(rele1, OUTPUT);
  pinMode(rele2, OUTPUT);
}

// === Pulsos individuales ===
void step_motor_direct(uint8_t motor_id, bool direction, int delay_us) {
  uint8_t pul_pin, dir_pin;

  if (motor_id == 1) {
    pul_pin = M1_PUL_PIN;
    dir_pin = M1_DIR_PIN;
  } else {
    pul_pin = M2_PUL_PIN;
    dir_pin = M2_DIR_PIN;
  }

  digitalWrite(dir_pin, direction ? HIGH : LOW);
  digitalWrite(pul_pin, HIGH);
  delayMicroseconds(delay_us);
  digitalWrite(pul_pin, LOW);
  delayMicroseconds(delay_us);
}

// === Movimiento genérico XY ===
void movement_xy(float dy_mm, float dx_mm, int delay_us) {
  float circ_mm = M_PI * DIAMETRO_POLEA_MM;
  float revA = (-dx_mm + dy_mm) / circ_mm;
  float revB = ( dx_mm + dy_mm) / circ_mm;

  int32_t stepsA = (int32_t)(revA * MICROSTEPS_PER_REV);
  int32_t stepsB = (int32_t)(revB * MICROSTEPS_PER_REV);

  bool dirA = stepsA >= 0;
  bool dirB = stepsB >= 0;

  stepsA = abs(stepsA);
  stepsB = abs(stepsB);

  int32_t max_steps = max(stepsA, stepsB);

  for (int32_t i = 0; i < max_steps; i++) {
    if (i < stepsA) step_motor_direct(1, dirA, delay_us);
    if (i < stepsB) step_motor_direct(2, dirB, delay_us);
  }
}

void movement_fast(float dx_mm, float dy_mm) {
  movement_xy(dx_mm, -dy_mm, 70);
}
void movement_slow(float dx_mm, float dy_mm) {
  movement_xy(dx_mm, -dy_mm, 120);
}

void moveToXY(float x_target, float y_target, bool slow = false) {
  if (x_target < X_MIN_MM){
    x_target = X_MIN_MM;
  }
  if (y_target < Y_MIN_MM){
    y_target = Y_MIN_MM;
  }
  if (x_target > X_MAX_MM ){
    x_target = X_MAX_MM ;
  }
  if (y_target > Y_MAX_MM ){
    y_target = Y_MAX_MM;
  }
  float dx = x_target - x_pos_mm;
  float dy = y_target - y_pos_mm;

  if (slow) {
    movement_slow(dx, dy);
  } else {
    movement_fast(dx, dy);
  }

  x_pos_mm = x_target;
  y_pos_mm = y_target;
}

void homeX() {
  float step_size_mm = 0.1f;
  while (digitalRead(HOMING_X_BUTTON_PIN) == LOW) {
    movement_fast(-step_size_mm, 0.0f);
    x_pos_mm -= step_size_mm;
    delay(1);
  }
  x_pos_mm = 0.0f;
  delayd(200);
}
void homeY() {
  float step_size_mm = 0.1f;
  while (digitalRead(HOMING_Y_BUTTON_PIN) == LOW) {
    movement_fast(0.0f, -step_size_mm);
    y_pos_mm -= step_size_mm;
    delay(1);
  }
  y_pos_mm = 0.0f;
  delayd(200);
}

// === Procesamiento de detecciones ===
void procesar_detecciones(String payload) {
  JSONVar data = JSON.parse(payload);
  if (!data.hasOwnProperty("detecciones")) return;

  Serial.println("Procesando detecciones:");
  for (int i = 0; i < data["detecciones"].length(); i++) {
    int x = int(data["detecciones"][i]["x"]);
    int y = int(data["detecciones"][i]["y"]);
    int clase = int(data["detecciones"][i]["clase"]);

    Serial.printf(" --- Objeto %d: clase=%d, x=%d, y=%d\n", i, clase, x, y);

    float xf = -15+(x / 100.0f); // CORRECCION DE CENTRO MAQUINA
    float yf = 42+(y / 100.0f); // CORRECCION DE CENTRO MAQUINA
    moveToXY(xf, yf);
    delayd(200);
    moveServoSlowly(113, 89, 20);
    delayd(400);
    //LED 1 LED 2
    digitalWrite(rele1, HIGH);
    digitalWrite(rele2, LOW);
    delayd(1000);
    moveServoSlowly(89, 113, 20);
    delayd(200);

    if(int(clase) == 1){
      moveToXY(5.0f, 0.0f);
    }
    else if(clase == 2 ){
      moveToXY(120.0f, 0.0f);
    }
    else{
      moveToXY(200.0f, 0.0f);
    }
    moveServoSlowly(113, 89, 20);
    digitalWrite(rele1, LOW);
    digitalWrite(rele2, HIGH);
    delayd(500);
    moveServoSlowly(89, 113, 20);
  }

  homing_realizado = false;
  ya_enviado_1 = false;
}

// === Evento WebSocket ===
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_CONNECTED) {
    client_num = num;
    cliente_conectado = true;
    Serial.printf("[WS] Cliente #%u conectado.\n", num);
  }
  else if (type == WStype_TEXT) {
    String msg = String((char*)payload);
    Serial.println("[WS] JSON recibido:");
    Serial.println(msg);
    procesar_detecciones(msg);
    esperando_datos = false;
  }
}

// === Setup general ===
void setup() {
  Serial.begin(9600);
  setup_motor_pins();
  Serial.println("iniciado");

  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n WiFi conectado. IP:");
  Serial.println(WiFi.localIP());

  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  miServo.attach(SERVO_PIN);

}

// === Loop principal ===
void loop() {
  webSocket.loop();

  if (!homing_realizado && cliente_conectado) {
    Serial.println("Procesing HOME data");
    homeX(); Serial.println("homeX");
    homeY(); Serial.println("homeY");
    homing_realizado = true;
  }

  if (homing_realizado && !ya_enviado_1 && cliente_conectado) {
    Serial.println(" Enviando la señal '1' a Raspberry Pi");
    webSocket.sendTXT(client_num, "1");
    ya_enviado_1 = true;
    esperando_datos = true;
    t_inicio_espera = millis();
  }

  if (esperando_datos && millis() - t_inicio_espera > 4000) {
    Serial.println("waiting for JSON from the Raspberry Pi");
    // Espera activa a que lleguen datos
  }
}
