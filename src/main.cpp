/**
 * @file main.cpp
 * @brief Sistema de Controle PID e Supervisório de Processos com Multi-function Shield (MFS)
 * @details 
 * Este firmware implementa um controlador PID reativo e um painel de supervisão utilizando
 * conceitos avançados de sistemas embarcados: Máquinas de Estados Finitas (FSM) e 
 * escalonamento cooperativo assíncrono (sem uso de delays bloqueantes).
 * 
 * Arquitetura de Software:
 * - O sistema é dividido em "Tasks" baseadas em millis().
 * - Classes abstratas/Structs são utilizadas para gerenciar Periféricos (Display, PID, Botões).
 * 
 * SINTONIA PID SUGERIDA (Método de tentativa e erro para inércia térmica simulada):
 * - Kp (Ganho Proporcional): 2.5 - Fornece uma resposta inicial rápida.
 * - Ki (Ganho Integral): 0.1 - Corrige o erro de estado estacionário a longo prazo.
 * - Kd (Ganho Derivativo): 1.0 - Suaviza a aproximação e previne overshoot excessivo.
 */

#include <Arduino.h>

// ==============================================================================
// DEFINIÇÕES E MAPEAMENTO DE HARDWARE (MFS)
// ==============================================================================
#define LATCH_PIN 4
#define CLOCK_PIN 7
#define DATA_PIN 8

#define PIN_BTN1 A1
#define PIN_BTN2 A2
#define PIN_BTN3 A3

#define PIN_LED1 13 // Heartbeat
#define PIN_LED2 12 // Indicador Automático
#define PIN_LED3 11 // Atuador PWM (0-255)
#define PIN_LED4 10 // Indicador de Alarme

#define PIN_BUZZER 3

#define PIN_TRIMPOT A0
#define PIN_SENSOR A4

// MFS usa lógica invertida (Active LOW) para LEDs e Buzzer
#define LED_ON  LOW
#define LED_OFF HIGH
#define BUZZER_ON LOW
#define BUZZER_OFF HIGH

// Descomente para simular a planta térmica via software (caso não haja LM35 ligado no A4)
#define SIMULE_THERMAL_PLANT

// ==============================================================================
// ESTRUTURAS DE DADOS E CLASSES (Arquitetura)
// ==============================================================================

/**
 * @brief Classe gerenciadora do Display de 7 Segmentos do MFS (Manipulação via Shift Register)
 */
class Display7Seg {
private:
    uint8_t currentDigit = 0;
    uint8_t displayData[4] = {0xFF, 0xFF, 0xFF, 0xFF}; // Tudo desligado inicialmente

    // Tabela de segmentos para dígitos de 0 a 9 (Anodo Comum do MFS - Lógica Invertida)
    const uint8_t SEGMENTS[11] = {
        0xC0, // 0
        0xF9, // 1
        0xA4, // 2
        0xB0, // 3
        0x99, // 4
        0x92, // 5
        0x82, // 6
        0xF8, // 7
        0x80, // 8
        0x90, // 9
        0xFF  // 10 (Espaço/Desligado)
    };

    // Máscaras para selecionar qual dos 4 dígitos acender
    const uint8_t DIGITS[4] = {0xF1, 0xF2, 0xF4, 0xF8}; 

public:
    void init() {
        pinMode(LATCH_PIN, OUTPUT);
        pinMode(CLOCK_PIN, OUTPUT);
        pinMode(DATA_PIN, OUTPUT);
    }

    // Configura um valor no display (Ex: numero entre 0 e 9999)
    void showNumber(int number) {
        if (number < 0) number = 0;
        if (number > 9999) number = 9999;

        displayData[0] = SEGMENTS[(number / 1000) % 10];
        displayData[1] = SEGMENTS[(number / 100) % 10];
        displayData[2] = SEGMENTS[(number / 10) % 10];
        displayData[3] = SEGMENTS[number % 10];

        // Apagar zeros à esquerda
        if (number < 1000) displayData[0] = 0xFF;
        if (number < 100)  displayData[1] = 0xFF;
        if (number < 10)   displayData[2] = 0xFF;
    }

    // Tarefa de multiplexação (deve ser chamada a cada 2-5ms)
    void multiplexTask() {
        digitalWrite(LATCH_PIN, LOW);
        shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, displayData[currentDigit]); // Envia os segmentos
        shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, DIGITS[currentDigit]);      // Envia seleção do dígito
        digitalWrite(LATCH_PIN, HIGH);

        currentDigit++;
        if (currentDigit >= 4) currentDigit = 0;
    }
};

/**
 * @brief Classe de Botão com Debounce por Software sem bloqueio
 */
class DebouncedButton {
private:
    uint8_t pin;
    bool lastState;
    bool state;
    uint32_t lastDebounceTime;
    uint32_t debounceDelay = 50; // ms

public:
    DebouncedButton(uint8_t p) : pin(p), lastState(HIGH), state(HIGH), lastDebounceTime(0) {}

    void init() {
        pinMode(pin, INPUT_PULLUP);
    }

    // Retorna true APENAS no momento em que é pressionado (falling edge)
    bool isPressed() {
        bool reading = digitalRead(pin);
        bool pressedEvent = false;

        if (reading != lastState) {
            lastDebounceTime = millis();
        }

        if ((millis() - lastDebounceTime) > debounceDelay) {
            if (reading != state) {
                state = reading;
                // Como tem PULLUP, LOW significa pressionado
                if (state == LOW) {
                    pressedEvent = true; 
                }
            }
        }
        lastState = reading;
        return pressedEvent;
    }
};

/**
 * @brief Implementação Clássica de um Controlador PID
 */
class PIDController {
public:
    float Kp, Ki, Kd;
    float integral, prevError;
    float minOutput, maxOutput;

    PIDController(float p, float i, float d, float minOut, float maxOut) :
        Kp(p), Ki(i), Kd(d), integral(0), prevError(0), minOutput(minOut), maxOutput(maxOut) {}

    void reset() {
        integral = 0;
        prevError = 0;
    }

    float compute(float setpoint, float processValue, float dt) {
        float error = setpoint - processValue;
        integral += error * dt;

        // Anti-windup da integral
        if (integral > maxOutput) integral = maxOutput;
        else if (integral < minOutput) integral = minOutput;

        float derivative = (error - prevError) / dt;
        float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

        // Limitando saída
        if (output > maxOutput) output = maxOutput;
        else if (output < minOutput) output = minOutput;

        prevError = error;
        return output;
    }
};

// ==============================================================================
// MÁQUINA DE ESTADOS E VARIÁVEIS GLOBAIS DO PROCESSO
// ==============================================================================

// Instanciamento de Periféricos e Controladores
Display7Seg display;
DebouncedButton btn1(PIN_BTN1);
DebouncedButton btn2(PIN_BTN2);
DebouncedButton btn3(PIN_BTN3);

// Sintonia sugerida: P=2.5, I=0.1, D=1.0 (Saída 0% a 100%)
PIDController pid(2.5, 0.1, 1.0, 0, 100);

// Enum para Máquina de Estados da Tela
enum DisplayState { SHOW_SETPOINT, SHOW_PROCESS_VAL, SHOW_PID_OUTPUT };
DisplayState currentScreen = SHOW_SETPOINT;

// Variáveis de Processo e Supervisório
float setpoint = 0.0;     // 0 a 100 graus
float processValue = 0.0; // 0 a 100 graus
float pidOutput = 0.0;    // 0 a 100 % (Porcentagem do atuador)

bool isAutoMode = true;   // true = PID Automático, false = Manual (saída zero)
bool alarmActive = false;
bool alarmAcknowledged = false;

// Controle do Buzzer Assíncrono
uint32_t buzzerEndTime = 0;
bool isBuzzing = false;

// ==============================================================================
// FUNÇÕES AUXILIARES E DE SISTEMA
// ==============================================================================

// Dispara um bipe curto no buzzer sem usar delay()
void shortBeep() {
    if(!isBuzzing) {
        digitalWrite(PIN_BUZZER, BUZZER_ON);
        buzzerEndTime = millis() + 50; // Bipe de 50ms
        isBuzzing = true;
    }
}

// ==============================================================================
// SETUP INICIAL
// ==============================================================================
void setup() {
    Serial.begin(115200);

    // Inicialização do Hardware
    display.init();
    btn1.init();
    btn2.init();
    btn3.init();

    pinMode(PIN_LED1, OUTPUT);
    pinMode(PIN_LED2, OUTPUT);
    pinMode(PIN_LED3, OUTPUT);
    pinMode(PIN_LED4, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);

    // Desliga todos os atuadores e sinalizadores
    digitalWrite(PIN_LED1, LED_OFF);
    digitalWrite(PIN_LED2, LED_OFF);
    digitalWrite(PIN_LED3, LED_OFF);
    digitalWrite(PIN_LED4, LED_OFF);
    digitalWrite(PIN_BUZZER, BUZZER_OFF);

    // Setup de Sinais Analógicos
    pinMode(PIN_TRIMPOT, INPUT);
    pinMode(PIN_SENSOR, INPUT);

    Serial.println("=============================================");
    Serial.println(" Sistema PID & Supervisorio MFS Iniciado");
    Serial.println("=============================================");
}

// ==============================================================================
// LOOP PRINCIPAL E ESCALONADOR COOPERATIVO
// ==============================================================================
void loop() {
    uint32_t currentMillis = millis();

    // ---------------------------------------------------------
    // TAREFA 1: Múltiplexação do Display (A cada 4ms)
    // ---------------------------------------------------------
    static uint32_t lastDisplayTime = 0;
    if (currentMillis - lastDisplayTime >= 4) {
        lastDisplayTime = currentMillis;
        display.multiplexTask();
    }

    // ---------------------------------------------------------
    // TAREFA 2: Leitura de Botões e Máquina de Estados (Cada 20ms)
    // ---------------------------------------------------------
    static uint32_t lastBtnTime = 0;
    if (currentMillis - lastBtnTime >= 20) {
        lastBtnTime = currentMillis;

        // BOTÃO 1: Alterna telas do Display
        if (btn1.isPressed()) {
            shortBeep();
            if (currentScreen == SHOW_SETPOINT) currentScreen = SHOW_PROCESS_VAL;
            else if (currentScreen == SHOW_PROCESS_VAL) currentScreen = SHOW_PID_OUTPUT;
            else currentScreen = SHOW_SETPOINT;
        }

        // BOTÃO 2: Alterna entre Modo Automático (Malha Fechada) e Manual
        if (btn2.isPressed()) {
            shortBeep();
            isAutoMode = !isAutoMode;
            if(!isAutoMode) {
                pidOutput = 0; // Zera saída caso mude pra manual
                pid.reset();   // Reseta a integral do PID para não acumular
            }
        }

        // BOTÃO 3: Reconhecimento de Alarme (Acknowledge)
        if (btn3.isPressed()) {
            shortBeep();
            if (alarmActive) {
                alarmAcknowledged = true;
            }
        }
    }

    // ---------------------------------------------------------
    // TAREFA 3: Malha de Controle PID e Sensores (A cada 100ms)
    // ---------------------------------------------------------
    static uint32_t lastCtrlTime = 0;
    if (currentMillis - lastCtrlTime >= 100) {
        float dt = (currentMillis - lastCtrlTime) / 1000.0; // Delta Time em segundos
        lastCtrlTime = currentMillis;

        // 1. LER SETPOINT DO TRIMPOT (mapeia 0-1023 para 0-100)
        int rawPot = analogRead(PIN_TRIMPOT);
        setpoint = map(rawPot, 0, 1023, 0, 100);

        // 2. LER VARIÁVEL DE PROCESSO (PV)
#ifdef SIMULE_THERMAL_PLANT
        // Simulação de inércia térmica da planta caso não haja sensor
        // PV tende ao valor da Saída (0 a 100) a uma taxa dependente da inércia
        float constInercia = 0.05; // Fator de acoplamento térmico (0.01 = lento, 0.1 = rápido)
        float dissipacao = 0.01;   // Simula perda de calor pro ambiente (Ambiente considerado = 0°C relat.)
        
        processValue += ((pidOutput * constInercia) - (processValue * dissipacao)) * dt;
#else
        // Lê sensor LM35 real no pino A4 (10mV por grau Celsius)
        int rawTemp = analogRead(PIN_SENSOR);
        float voltage = rawTemp * (5.0 / 1023.0);
        processValue = voltage * 100.0; 
#endif

        // 3. CALCULAR PID SE EM MODO AUTO
        if (isAutoMode) {
            pidOutput = pid.compute(setpoint, processValue, dt);
        }

        // 4. ATUALIZAR RECURSOS DE CONFORTO VISUAL (TELA)
        int numberToDisplay = 0;
        switch (currentScreen) {
            case SHOW_SETPOINT:     numberToDisplay = (int)setpoint; break;
            case SHOW_PROCESS_VAL:  numberToDisplay = (int)processValue; break;
            case SHOW_PID_OUTPUT:   numberToDisplay = (int)pidOutput; break;
        }
        display.showNumber(numberToDisplay);

        // 5. CHECAR ALARME (Erro > 15%)
        float err = abs(setpoint - processValue);
        if (err > 15.0) {
            alarmActive = true;
        } else {
            // Se erro caiu pra zona segura, reseta o mecanismo de alarme automaticamente
            alarmActive = false;
            alarmAcknowledged = false; 
        }

        // 6. ATUALIZAR ATUADOR (LED3 SIMULANDO RESISTÊNCIA DE AQUECIMENTO)
        // Mapeia 0-100% de pidOutput para 0-255 no timer de PWM
        // OBS: Como LEDs do MFS são active LOW, 255-0 invés de 0-255 pra brilho
        int pwmValue = map((int)pidOutput, 0, 100, 0, 255);
        analogWrite(PIN_LED3, 255 - pwmValue); // Duty cycle invertido no LED

        // 7. ATUALIZAR INDICADOR DE MODO
        digitalWrite(PIN_LED2, isAutoMode ? LED_ON : LED_OFF);

        // Log para calibração serial
        Serial.print("SP:"); Serial.print(setpoint);
        Serial.print(" PV:"); Serial.print(processValue);
        Serial.print(" OUT(%):"); Serial.print(pidOutput);
        Serial.print(" AUTO:"); Serial.println(isAutoMode);
    }

    // ---------------------------------------------------------
    // TAREFA 4: Tratamento de Alarmes e Buzzer Assíncrono
    // ---------------------------------------------------------
    
    // Processamento do Heartbeat (Pisca à 1Hz) e Led de Alarme Flashing
    static uint32_t lastBlinkTime = 0;
    static bool blinkState = false;
    if (currentMillis - lastBlinkTime >= 500) {
        lastBlinkTime = currentMillis;
        blinkState = !blinkState;
        
        digitalWrite(PIN_LED1, blinkState ? LED_ON : LED_OFF); // Heartbeat D1

        if (alarmActive) {
            digitalWrite(PIN_LED4, blinkState ? LED_ON : LED_OFF); // D4 pisca com alarme
            
            // Toca a sirene intermitente se não foi reconhecida no botão A3
            if (!alarmAcknowledged && blinkState) {
                shortBeep();
            }
        } else {
            digitalWrite(PIN_LED4, LED_OFF);
        }
    }

    // Processamento temporal do encerramento do toque do buzzer
    if (isBuzzing && currentMillis > buzzerEndTime) {
        digitalWrite(PIN_BUZZER, BUZZER_OFF);
        isBuzzing = false;
    }
}