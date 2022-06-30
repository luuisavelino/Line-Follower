#include <AFMotor.h>
#include <TimerOne.h>

AF_DCMotor motor_esq(1);
AF_DCMotor motor_dir(4);
 
// ==================== Variáveis ====================
//
// ------------------- Sensores -------------------
int SENSOR1, SENSOR2, SENSOR3, SENSOR4, SENSOR5;
int sensor1_antigo = HIGH, sensor2_antigo = HIGH;
//
// ------------------- Angulos -------------------
float angulo_normalizado;
int setpoint = 30;
//
// ------------------- Erros -------------------
float I_Error = 0, D_Error = 0;
float lastError = 0, erro = 0;
//
// ------------------- Ganhos -------------------
//
// ==== Ganho com o método de sintonia ZN ====
float Kp = 10; 
float Ki = 10; 
float Kd = 10;
//
// ==== Ganho com o método de sintonia IAE ===
// float Kp = ;
// float Ki = ;
// float Kd = ;
//
// ==== Ganho com o método de sintonia ITAE ==
// float Kp = ; 
// float Ki = ;
// float Kd = ;
//
//--------------------------------------------
//
// ------------------- Controle -------------------
float out_Proportional, out_Integrative, out_Derivative;
float out_PID, u_control;
//
// ------------------- PWM -------------------
float duty_left_motor, duty_right_motor;
//
// ------------------- Saidas -------------------
//
float tensao_min = 0, tensao_max = 5;
// ==================================================


float get_value(int sensor1, int sensor2, int sensor3, int sensor4, int sensor5, int sensor1_antigo = HIGH, int sensor2_antigo = HIGH)
{
    if (sensor1 == LOW) {
        return 14.4759;
    } else if (sensor1 == LOW && sensor2 == LOW) {
        return 18.9965;
    } else if (sensor2 == LOW) {
        return 23.6598;
    } else if (sensor2 == LOW && sensor3 == LOW) {
        return 26.8202;
    } else if (sensor3 == LOW) {
        return 30.0000;
    } else if (sensor3 == LOW && sensor4 == LOW) {
        return 33.1798;
    } else if (sensor4 == LOW) {
        return 36.3402;
    } else if (sensor4 == LOW && sensor5 == LOW) {
        return 41.0035;
    } else if (sensor5 == LOW) {
        return 45.5241;
    } else if (sensor1 == HIGH && sensor2 == HIGH && sensor3 == HIGH && sensor4 == HIGH && sensor5 == HIGH && sensor1_antigo == LOW) {
        return 60;
    } else if (sensor1 == HIGH && sensor2 == HIGH && sensor3 == HIGH && sensor4 == HIGH && sensor5 == HIGH && sensor5_antigo == LOW) {
        return 0.0;
    } else {
        return 30;
    }
}

float coleta_dados() {
    SENSOR1 = digitalRead(1);
    SENSOR2 = digitalRead(2);
    SENSOR3 = digitalRead(3);
    SENSOR4 = digitalRead(4);
    SENSOR5 = digitalRead(5);

    angulo_normalizado = get_value(SENSOR1, SENSOR2, SENSOR3, SENSOR4, SENSOR5, sensor1_antigo, sensor2_antigo);

    return angulo_normalizado, SENSOR1, SENSOR5;
}

float calcula_erros(float setpoint, float valor_atual, float lastError) {
    erro = float(setpoint - valor_atual);
    I_Error += erro * 0.02;
    D_Error = float((lastError - erro) / 0.02);

    return erro, I_Error, D_Error;
}

float calcula_controle(float erro, float I_Error, float D_Error) {
    out_Integrative = Ki * I_Error;

    if (out_Integrative > tensao_max) {
        out_Integrative = tensao_max;
    } else if (out_Integrative < tensao_min) { 
        out_Integrative = tensao_min;
    }

    out_Derivative = Kd * D_Error;
    out_Proportional = Kp * erro;

    out_PID = out_Proportional + out_Integrative + out_Derivative;

    if (out_PID > tensao_max) { 
        out_PID = tensao_max;
    } else if (out_PID < tensao_min) {
        out_PID = tensao_min;
    }

    u_control = (out_PID / tensao_max) * 511;

    duty = int(u_control);
    if (duty > 511) {
        duty = 511;
    } else if (duty < 0) {
        duty = 0;
    }

    return u_control, erro;
}

float motor_speed(int duty) {

    left_motor = 510 - duty;
    right_motor = duty

    if (left_motor > 255) {
        left_motor = 255;
    }

    if (right_motor > 255) {
        right_motor = 255;
    }
}

void setup() {
  Timer1.initialize(20000);
  Timer1.attachInterrupt(amostragem);
}

void amostragem() {
    angulo_normalizado, sensor1_antigo, sensor2_antigo = coleta_dados();

    erro, I_Error, D_Error = calcula_erros(setpoint, angulo_normalizado, lastError);

    duty, lastError = calcula_controle(erro, I_Error, D_Error);
}

void loop()
{
    duty_left_motor, duty_right_motor = motor_speed(duty);

    motor_esq.setSpeed(duty_left_motor);
    motor_esq.run(FORWARD);
    motor_dir.setSpeed(duty_right_motor);
    motor_dir.run(FORWARD);

    delay(25);
}
