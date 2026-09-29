#include "HardwareSerial.h"
#include <Arduino.h>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <ratio>
#include "DHTesp.h"
#include <iostream>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "config.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306>

const uint8_t ANCHO_PANTALLA = 128;
const uint8_t ALTO_PANTALLA = 64;
const int8_t OLED_RESET = -1;
const uint8_t DIRECCION_OLED = 0x3C;

Adafruit_SSD1306 display(ANCHO_PANTALLA, ALTO_PANTALLA, &Wire, OLED_RESET); 
unsigned long muestraPantalla = 0; 
const unit muestreoPantalla = 1000; 

WiFiClient red;
PubSubClient mqtt(red);

String clientId, topicDatos, topicEstado, topicCmd;

uint32_t tWiFi = 0;
uint32_t tReconexion = 0;
uint32_t tPub = 0;
uint32_t esperaReconexion = 2000;
const uint32_t ESPERA_MAXIMA_MQTT = 30000;
const uint32_t REINTENTO_WIFI_MS = 15000;
const uint32_t PERIODO_PUB_MS = 10000;

DHTesp dhtSensor;
const byte DHT_PIN = 15;
const byte MQ = 34;
const byte buzzer = 33;
const byte canalBuzzer = 0;
const byte IR = 35;

enum ESTADO {VIGILANCIA, SOSPECHA, ALERTA_CONFIRMADA, ERROR};
ESTADO estadoActual = VIGILANCIA;
ESTADO anterior;
//Codigo dado por el profesor
//=================================================================================
const uint8_t N_FILTRO = 5;
const bool USAR_MEDIANA = false;
const float V_REF        = 3.3f;       // [V] tension de referencia del ADC
const uint16_t CUENTAS_MAX = 4095;     // ADC de 12 bits: 0 a 4095
const float ESCALA_SENSOR = -0.36f;    // [grados C / V] sensibilidad nominal
const float OFFSET_SENSOR = 0.65f;

//Estado interno
//=================================================================================
int t_ultima_muestra = 0;
float ventana[N_FILTRO];
byte idx_ventana = 0;
int indice =0;
bool ventana_llena = false;


//Contador de muestras validas e invalidas
//============================================================================
unsigned DHTmuestrasValidas;
unsigned MQmuestrasValidas;
unsigned IRmuestrasValidas;
unsigned DHTmuestrasInvalidas;
unsigned DHTlocalesInvalidas;
unsigned MQmuestrasInvalidas;
unsigned IRMuestasInvalidas;


//Tiempo de muestreo de cada sensor
//============================================================================
const int muestreoDHT = 2000; //valor normal de 60000
const int muestreoIR = 500;
const int muestreoMQ = 3000;

//Contadores de muestreo para cada sensor
//============================================================================
unsigned long muestraMQ;
unsigned long muestraIR;
unsigned long muestraDHT;


//Tiempos de espera
//============================================================================
const int tiempoMaximo = 100000;
const int actualizacion = 600000;


//Funciones basicas para el sensor MQ-2
//============================================================================
//Declaraciones necesarias para funciones del sensor MQ-2
const float VC_MV = 5000.0;
const float RL_KOHM = 2.0;

float resistenciaMQ = -1.0; 

int leer_mv(){
    long suma = 0;
    for (int i= 0; i < 5; i++) suma += analogReadMilliVolts(MQ);
    return (int)(suma/5);
}
int filtrar(int valor) {                  // media movil de N_FILTRO muestras
    ventana[indice] = valor;
    indice = (indice + 1) % N_FILTRO;
    if (indice == 0) ventana_llena = true;

    int tope = ventana_llena ? N_FILTRO : indice;
    long suma = 0;
    for (int i = 0; i < tope; i++) suma += ventana[i];
    return (int)(suma / tope);
    }
float resistencia_kohm(int mv) {          // divisor RS-RL del modulo
    if (mv <= 0) return -1.0;               // lectura invalida
    return RL_KOHM * (VC_MV - mv) / mv;
}


//Funciones para el sensor DHT-22
//============================================================================
//Inicio de variables globales para dht
float temp, tempAnterior, humedad, tempActual;
void lecturasDHT(){
	TempAndHumidity data = dhtSensor.getTempAndHumidity();
	temp = data.temperature;
	humedad = data.humidity;
	if(isnan(temp) || isnan(humedad)){
		DHTlocalesInvalidas+=1;
		DHTmuestrasInvalidas+=1;
		Serial.println("lectura invalida DHT");
		return;
	}
	Serial.printf("La temperatura es: %.2f y la humedad es: %.2f\n", temp, humedad);
}

void actualizacionTemp(float temp){
	if (isnan(temp)) tempActual = temp;
	Serial.print("La temperatura actual es: ");
	Serial.println(tempActual);
}

float resta() {
	return tempActual - tempAnterior;
}
//Funciones para el sensor IR
//============================================================================
int desc;
int nivelIR(){
    int lectura = digitalRead(IR);
	Serial.printf("El sensor IR esta actualmente en: %d\n", lectura);
    return lectura;
}


//Funciones Generales
//============================================================================
void lecturaMQ(){
	int suma = leer_mv();
	int filtrado = filtrar(suma);
	resistenciaMQ = resistencia_kohm(filtrado);
	if (resistenciaMQ != -1.0) MQmuestrasValidas +=1;
	else MQmuestrasInvalidas +=1;
	Serial.printf("Los datos de MQ son: %d, %.2f.\n", filtrado, resistenciaMQ);
}

/*
bool isEsperaMaxima(){
    static unsigned long tiempo;
    static bool flag = false;
    if (estadoActual == SOSPECHA){
        tiempo += millis();
        flag = false;
    	if (tiempo >= tiempoMaximo){
        	tiempo = 0;
        	flag = true;
    	}
	}
	return flag;
}
*/
bool isEsperaMaxima() {
    static unsigned long tiempoInicio = 0;
    static bool midiendo = false;

    if (estadoActual == SOSPECHA) {
        // Registra la marca de tiempo solo la primera vez que entra a este estado
        if (!midiendo) {
            tiempoInicio = millis();
            midiendo = true;
        }

        // Calcula la diferencia para ver si se alcanzó el tiempo máximo
        if (millis() - tiempoInicio >= tiempoMaximo) {
            midiendo = false; // Se reinicia el estado para futuros usos
            return true;
        }
    } else {
        // Si el estado cambia a VIGILANCIA o ALERTA, se aborta y reinicia el temporizador
        midiendo = false;
    }

    return false;
}

/*
void avisar(bool encender) {              // funcion para buzzer en caso de que placa no acepte ledc
  if (encender) tone(buzzer, 2000);
  else noTone(buzzer);
}
*/

void avisar(bool encender){         //funcion para buzzer / ledc
    if (encender) {
    ledcWriteTone(canalBuzzer, 2000);     // Emite el tono de 2000 Hz
    } else {
    ledcWriteTone(canalBuzzer, 0);        // Frecuencia 0 para silenciar ((equivale a noTone))
  }
}

const char* cambiarNombre(ESTADO e) {
  switch (e) {
    case VIGILANCIA: return "VIGILANDO";
    case SOSPECHA:  return "SOSPECHA";
    case ALERTA_CONFIRMADA: return "ALERTA";
    case ERROR: return "ERROR";
  }
  return "?";
}

//Funcion para poder ver el cambio de nombre a lo largo de la comunicacion serial
void cambioEstado(ESTADO actual){
    estadoActual = actual;
    if (actual != anterior){
		DHTlocalesInvalidas = 0;
        ESTADO temp_anterior = anterior; 
		anterior = actual;
		Serial.printf("Estado anterior: %s | Estado actual: %s\n", cambiarNombre(temp_anterior), cambiarNombre(actual));
	    
        if (mqtt.connected()) {
            mqtt.publish(topicEstado.c_str(), cambiarNombre(actual), true);
        }
    }
}

bool lectura = true;

void actualizarPantalla(bool, fuegoActivo) {
 	display.clerDisplay(); 
	display.setTextSize(1);
	display.setTextColor(SSD1306_WHITE); 

	display.setCursor(0, 0); 
	display.printf("Modo: %s", cambiarNombre(estadoActual)); 
	display.drawLine(0, 10, 128, 10, SSD1306_WHITE); 

	display.setCursor(0, 15); 
	display.printf("Temp: %.1f C", temp);  
	display.setCursor(0, 25); 
	display.printf("Hum: %.1f %%"; humedad); 

	display.setCursor(0, 35); 
	if (resistenciaMQ < 0) {
		display.print("Gas: invalido"); 
	} else {
		display.printf("Gas Rs: %.2f kOhm", resistenciaMQ);
	}
	display.setCursor(0, 45); 
	display.printf("Llama: %s"; fuegoActivo? "FUEGO!!" : "OK");

	display.setCursor(0, 55); 
	display.printf("Wifi:%s MQTT:%s",
					(WiFi.status() == WL_CONNECTED)? "OK" : "NO", 
					mqtt.connectedd() ? "OK" : "NO");
	display.displsy(); 
}

void mantenerWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  
  uint32_t ahora = millis();
  if (ahora - tWiFi < REINTENTO_WIFI_MS) return;
  
  tWiFi = ahora;
  Serial.println("[wifi] Sin red, reintentando...");
  WiFi.reconnect(); 
}

void recibirComando(char* topic, byte* payload, unsigned int largo) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload, largo);
  if (error) {
    Serial.printf("[cmd] JSON invalido en %s: %s\n", topic, error.c_str());
    return;
  }
  Serial.printf("[cmd] recibido en %s\n", topic);
}

void mantenerMQTT() {
  if (mqtt.connected()) return;
  if (WiFi.status() != WL_CONNECTED) return; 
  
  uint32_t ahora = millis();
  if (ahora - tReconexion < esperaReconexion) return;
  tReconexion = ahora;

  Serial.printf("[mqtt] Conectando como %s ... ", clientId.c_str());
  
  if (mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS, topicEstado.c_str(), 1, true, "offline")) {
    Serial.println("OK");
    mqtt.publish(topicEstado.c_str(), "online", true); 
    mqtt.subscribe(topicCmd.c_str(), 1);               
    esperaReconexion = 2000;                           
  } else {
    Serial.printf("FALLO rc=%d\n", mqtt.state());
    esperaReconexion = (esperaReconexion * 2 > ESPERA_MAXIMA_MQTT) ? ESPERA_MAXIMA_MQTT : esperaReconexion * 2;
  }
}

void publicarDatos() {
  if (!mqtt.connected()) return;

  JsonDocument doc;
  bool sensorOk = (!isnan(temp) && !isnan(humedad));

  if (sensorOk) {
    doc["temperatura"] = roundf(temp * 10.0f) / 10.0f;
    doc["humedad"]     = roundf(humedad * 10.0f) / 10.0f;
  }
  
  doc["sensor_ok"] = sensorOk ? 1 : 0;
  doc["estado_sistema"] = cambiarNombre(estadoActual);
  doc["rssi_dbm"]  = WiFi.RSSI();

  char payload[256];
  size_t n = serializeJson(doc, payload, sizeof(payload));

  if (mqtt.publish(topicDatos.c_str(), (const uint8_t*)payload, n, false)) {
    Serial.printf("[pub] %s -> %s\n", topicDatos.c_str(), payload);
  } else {
    Serial.println("[pub] ERROR publish()");
  }
}

void setup(){
	Serial.begin(115200);
	clientId    = String(MQTT_USER) + "-" + NODO;
    topicDatos  = String("curso/") + MQTT_USER + "/" + PROYECTO + "/" + NODO; 
    topicEstado = topicDatos + "/estado";
    topicCmd    = topicDatos + "/cmd";

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  tWiFi = millis();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
	mqtt.setCallback(recibirComando);
  mqtt.setKeepAlive(15);
  mqtt.setSocketTimeout(3);
	
	dhtSensor.setup(DHT_PIN, DHTesp::DHT11);
	analogSetPinAttenuation(MQ, ADC_11db);
	pinMode(IR, INPUT_PULLUP);
	ledcSetup(canalBuzzer, 2000, 8);
	ledcAttachPin(buzzer, canalBuzzer);

	Wire.begim(21,22); 
	if (!display.begin(SSD1306_SWITCHCAPVCC, DIRECCION_OLED)) {
		Serial.println(" Error al iniciar"); 
	} else {
		display.claerDisplay(); 
		display.setTextSize(1); 
		display.setTextColor(SSD1306_WHITE); 
		display.setCursor(15, 25); 
		display.prinln("Iniciando Nodo..")
		display.display();

	}
	//llamada por primera vez para obtener las primeras lecturas
}

void loop(){
	mantenerWiFi();
	mantenerMQTT();
	mqtt.loop();

	unsigned long time = millis();
	static unsigned long ultimoTimeAc = 0; 
	static bool ac;
	static bool senal;

	if (time - tPub >= PERIODO_PUB_MS) {
    tPub = time;
    publicarDatos();
	}
	
	if (lectura){
		Serial.println("Primera Lectura");
		lecturasDHT();
		actualizacionTemp(temp);
		lecturaMQ();
		desc = nivelIR();
		lectura = false;
		Serial.printf("Estado actual: %s\n", cambiarNombre(estadoActual));
	}


	if (time - muestraMQ > muestreoMQ){
		muestraMQ = millis();
		lecturaMQ();
	}

	if (time - muestraDHT > muestreoDHT){
		muestraDHT = millis();
		lecturasDHT();
	}

	if (time - ultimoTimeAc > actualizacion)actualizacionTemp(temp);

	if (time - muestraIR > muestreoIR){
		muestraIR = millis();
		ac = nivelIR();
	}

	if (time - muestraPantalla > muestreoPantalla) {
		muestreo = millis(); 
		actualizarPantalla(ac == 0); 
	}

    static uint32_t t_pub = 0;
    const uint32_t PERIODO_PUB_MS = 10000; 

    if (time - t_pub > PERIODO_PUB_MS) {
        t_pub = time;

        JsonDocument doc;
        doc["temperatura"] = temp; 
        doc["humedad"] = humedad;
        doc["resistencia"] = resistenciaMQ;
        doc["fuego"] = (ac == 0); 
        
        char buf[256];
        serializeJson(doc, buf);

        if(mqtt.connected()){
            mqtt.publish(topicDatos.c_str(), (uint8_t*)buf, strlen(buf), true);
            Serial.print("JSON Publicado: ");
            Serial.println(buf);
        }
    }

	switch(estadoActual){
		case VIGILANCIA:{
			senal = false;
			if (resta() > 5 || ac != desc) cambioEstado(SOSPECHA);
			else if( resta() > 5 && ac != desc) cambioEstado(ALERTA_CONFIRMADA);
			else if(DHTlocalesInvalidas >= 3){
				DHTlocalesInvalidas = 0;
				cambioEstado(ERROR);
			}
			break;
		}
		case SOSPECHA:{
			senal = false;
			if (resta() > 5 && ac != desc) cambioEstado(ALERTA_CONFIRMADA);
			else if(isEsperaMaxima()) cambioEstado(VIGILANCIA);
			else if(DHTlocalesInvalidas >= 3){
				DHTlocalesInvalidas = desc;
				cambioEstado(ERROR);
			}
			break;
		}
		case ALERTA_CONFIRMADA:{
			senal = true;
			avisar(senal);
			if (resta() < 3 && ac == desc) cambioEstado(VIGILANCIA);
			else if(resta() < 5 || ac == desc) cambioEstado(SOSPECHA);
			else if(DHTlocalesInvalidas >= 3){
				DHTlocalesInvalidas = 0;
				cambioEstado(ERROR);
			}
			break;
		}
		case ERROR:{
			if (DHTlocalesInvalidas < 3 ) cambioEstado(VIGILANCIA);
			break;
		}
	}
}
