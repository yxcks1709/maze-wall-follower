/*
 * Maze Wall-Follower Robot (left-hand rule)
 * Board:   ROBOTIS OpenCM 9.04
 * Sensors: 3x OLLO IR sensors (front, left, right)
 * Motors:  2x Dynamixel in wheel mode (IDs 14 and 15)
 *
 * The robot follows the wall on its left. When the left wall ends it turns
 * left into the gap; when a wall appears in front it picks the freer side;
 * when it is boxed in it turns around.
 */

#include <OLLO.h>

#define DXL_BUS_SERIAL1 1

#define MOTOR_L 14
#define MOTOR_R 15

// IR sensor ports
#define IR_FRENTE 1   // front
#define IR_IZQ    2   // left
#define IR_DER    3   // right

// Sensor readings with nothing in front of them (baseline)
#define BASE_IZQ  162
#define BASE_DER   29

// Sensor thresholds
#define UMBRAL_FRENTE 1500   // above this = wall in front
#define IZQ_HUECO      220   // below this = no wall on the left (gap)
#define IZQ_LEJOS      350   // below this = drifted away -> steer gently left
#define IZQ_CERCA      600   // above this = too close on the left -> steer right
#define UMBRAL_DER     700   // above this = wall on the right (used for turn decisions)
#define DER_CERCA      800   // above this = too close on the right -> steer left
#define MARGEN          35   // how much busier the left must be to choose a right turn
#define AJUSTE_DER      15   // extra speed for the right wheel (compensates drift)

// Motion
#define VELOCIDAD            300   // cruise speed
#define VELOCIDAD_GIRO       200   // turning speed
#define CORRECCION_APARTAR   100   // correction to move away from a close wall
#define CORRECCION_ACERCAR    40   // gentle correction to move back toward the left wall
#define TIEMPO_90           1050   // ms for a 90° turn   -> calibrate
#define TIEMPO_GIRO         2100   // ms for a 180° turn  -> calibrate
#define AVANCE_ANTES        1000   // ms to clear the corner before turning -> tune
#define AVANCE_DESPUES       500   // ms to enter the new corridor -> tune

#define CONFIRMAR 5   // consecutive readings needed to confirm a gap

Dynamixel Dxl(DXL_BUS_SERIAL1);
OLLO myOLLO;

int  contadorIzq   = 0;      // consecutive readings without a left wall
bool teniaParedIzq = false;  // was the robot following a left wall?

// ---------- Motion helpers ----------

void avanzar() {
  Dxl.goalSpeed(MOTOR_L, VELOCIDAD);
  Dxl.goalSpeed(MOTOR_R, (VELOCIDAD + AJUSTE_DER) | 0x400);  // 0x400 = reverse direction
}

// Drive forward while keeping a safe distance from the walls
void avanzarCorrigiendo(int izq, int der) {
  int vL = VELOCIDAD;
  int vR = VELOCIDAD + AJUSTE_DER;

  if (izq >= IZQ_CERCA) {
    vR = VELOCIDAD - CORRECCION_APARTAR;    // too close to the left -> steer right
    SerialUSB.println("-> RIGHT (left too close)");
  }
  else if (der >= DER_CERCA) {
    vL = VELOCIDAD - CORRECCION_APARTAR;    // too close to the right -> steer left
    SerialUSB.println("-> LEFT (right too close)");
  }
  else if (izq >= IZQ_HUECO && izq < IZQ_LEJOS) {
    vL = VELOCIDAD - CORRECCION_ACERCAR;    // drifted from the left wall -> gently back
    SerialUSB.println("-> gentle left (left far)");
  }

  Dxl.goalSpeed(MOTOR_L, vL);
  Dxl.goalSpeed(MOTOR_R, vR | 0x400);
}

void parar() {
  Dxl.goalSpeed(MOTOR_L, 0);
  Dxl.goalSpeed(MOTOR_R, 0);
}

// Drive forward for a given time, stopping early if a wall appears in front
void avanzarTiempo(int tiempo) {
  avanzar();
  for (int t = 0; t < tiempo; t += 20) {
    if (myOLLO.read(IR_FRENTE, IR_SENSOR) >= UMBRAL_FRENTE) {
      break;
    }
    delay(20);
  }
  parar();
}

void girarDerecha(int tiempo) {
  Dxl.goalSpeed(MOTOR_L, VELOCIDAD_GIRO);
  Dxl.goalSpeed(MOTOR_R, VELOCIDAD_GIRO);
  delay(tiempo);
  parar();
}

void girarIzquierda(int tiempo) {
  Dxl.goalSpeed(MOTOR_L, VELOCIDAD_GIRO | 0x400);
  Dxl.goalSpeed(MOTOR_R, VELOCIDAD_GIRO | 0x400);
  delay(tiempo);
  parar();
}

// ---------- Setup / loop ----------

void setup() {
  Dxl.begin(3);
  Dxl.wheelMode(MOTOR_L);
  Dxl.wheelMode(MOTOR_R);

  myOLLO.begin(IR_FRENTE, IR_SENSOR);
  myOLLO.begin(IR_IZQ,    IR_SENSOR);
  myOLLO.begin(IR_DER,    IR_SENSOR);
}

void loop() {
  int frente = myOLLO.read(IR_FRENTE, IR_SENSOR);
  int izq    = myOLLO.read(IR_IZQ,    IR_SENSOR);
  int der    = myOLLO.read(IR_DER,    IR_SENSOR);

  SerialUSB.print("F: ");   SerialUSB.print(frente);
  SerialUSB.print("  L: "); SerialUSB.print(izq);
  SerialUSB.print("  R: "); SerialUSB.println(der);

  bool paredIzq    = (izq    >= IZQ_HUECO);
  bool paredFrente = (frente >= UMBRAL_FRENTE);
  bool paredDer    = (der    >= UMBRAL_DER);

  if (paredIzq) {
    teniaParedIzq = true;
    contadorIzq = 0;
  } else if (contadorIzq < CONFIRMAR) {
    contadorIzq++;   // capped: only needs to reach CONFIRMAR
  }

  if (paredFrente) {
    // Wall in front -> decide which way to turn
    parar();
    delay(200);

    int espacioIzq = izq - BASE_IZQ;
    int espacioDer = der - BASE_DER;

    if (paredIzq && paredDer) {
      SerialUSB.println("== U-TURN");
      girarIzquierda(TIEMPO_GIRO);     // boxed in -> turn around
    }
    else if (espacioIzq <= espacioDer + MARGEN) {
      SerialUSB.println("== 90 LEFT (wall in front)");
      girarIzquierda(TIEMPO_90);
      teniaParedIzq = false;
    }
    else {
      SerialUSB.println("== 90 RIGHT (wall in front)");
      girarDerecha(TIEMPO_90);         // left side busier -> turn right
    }
    contadorIzq = 0;
  }
  else if (teniaParedIzq && contadorIzq >= CONFIRMAR) {
    // The left wall ended -> gap -> turn left into it
    SerialUSB.println("== GAP ON THE LEFT");
    contadorIzq = 0;
    teniaParedIzq = false;
    avanzarTiempo(AVANCE_ANTES);       // clear the corner (stops if a wall appears)
    girarIzquierda(TIEMPO_90);
    avanzarTiempo(AVANCE_DESPUES);     // enter the new corridor (stops if a wall appears)
  }
  else {
    // Path is clear -> drive forward while correcting distance to the walls
    avanzarCorrigiendo(izq, der);
  }

  delay(10);
}
