//COLODNER, BOCCI, BRAVAR, TOLEDO --> GRUPO 2 5MB
//LIBRERIAS
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_AHT10.h>
//EEPROM
#include <Preferences.h>
//TELEGRAM
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
//MQTT
#include "AsyncMqttClient.h"
#include "time.h"
#include "Arduino.h"
//CONFIGURA LIQUID
LiquidCrystal_I2C lcd(0x27, 16, 2);

//CONFIGURA AHT10
Adafruit_AHT10 aht;

//TELEGRAM
//WIFI
const char* ssid = "MECA-IoT"; //"Personal-0AF 2.4GHz";
const char* password = "IoT$2026"; //"435799A0AF";
//Token del bot
#define BOTtoken "8025037753:AAFpbCTQcwS2Zl1ebt8SktN_9j35VvIw4xY"
//ID del chat con el bot
#define CHAT_ID "6235002183"
WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);
int checkeo_gas1 = 0;
int checkeo_gas2 = 0;
int checkeo_temp = 0;
int checkeo_hum = 0;
int checkeo_ldr = 0;

TaskHandle_t Task1;
TaskHandle_t Task2;


//EEPROM
Preferences VU_eeprom;

//ESTADOS
#define VAL_GAS1 1
#define VAL_GAS2 2
#define VAL_TEMP 3
#define VAL_HUM 4
#define VAL_LDR 5
#define VAL_GMT 6
#define VAL_RELE 7
#define GUARDADO 8

int ESTADO = VAL_GAS1;
int contando_select = 0;

//REGLONES LCD
String reglon1;
String reglon2;

//BOTONES
#define BOT_PANTALLA 17
#define BOT_SELECT 23
#define BOT_SUMA 15
#define BOT_RESTA 4
#define BOT_SAVE 16

unsigned long contando = 0;

//RELE
#define PIN_RELE 25
int estado_rele = 0;

//UMBRALES
int umbral_gas1 = 24;
int umbral_gas2 = 24;
int umbral_temp = 24;
int umbral_hum = 24;
int umbral_ldr = 24;

//SENSORES
#define MQA1 32
#define MQA2 33
#define LDR_PIN 35

int gas1 = 0;
int gas2 = 0;
int ldr = 0;

float lectemp = 0.0;
float lechum = 0.0;

bool botonESTADO(int pin);
void fun_lcd();
void MDE();

//MQTT
const char name_device = 22;
unsigned long now = millis();                ///valor actual
unsigned long lastMeasure1 = 0;              ///variable para contar el tiempo actual
unsigned long lastMeasure2 = 0;              ///variable para contar el tiempo actual
const unsigned long interval_envio = 30000;  //Intervalo de envio de datos mqtt
const unsigned long interval_leeo = 60000;   //Intervalo de lectura de datos y guardado en la cola
int i = 0;
int gmt_user = 3;
int gmt_offset = -10800;

///time
long unsigned int timestamp;  // hora
const char* ntpServer = "south-america.pool.ntp.org";
const long gmtOffset_sec = -10800;
const int daylightOffset_sec = 0;
///variables ingresar a la cola struct
int indice_entra = 0;
int indice_saca = 0;
bool flag_vacio = 1;
/////mqqtt
#define MQTT_HOST IPAddress(192, 168, 5, 123)  ///se debe cambiar por el ip de meca o hall del 4
#define MQTT_PORT 1884
#define MQTT_USERNAME "esp32"
#define MQTT_PASSWORD "mirko15"
char mqtt_payload[150];  /////
// Test MQTT Topic
#define MQTT_PUB "/esp32/datos_sensores"
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

typedef struct
{
  long time;
  float T1;     ///temp en grados
  float H1;     ///valor entre 0 y 99 // mojado es cercano al 100
  float luz;    ///valor entre 0 y 99 . si hay luz es cercano al 100
  float G1;     ///valor entre 0 y 99
  float G2;     ///valor entre 0 y 99
  bool oct;     ///Lectura del octoacoplador
  bool Alarma;  //

} estructura;
////////////////
const int valor_max_struct = 1000;          ///valor vector de struct
estructura datos_struct[valor_max_struct];  ///Guardo valores hasta que lo pueda enviar
estructura aux2;

void setupmqtt() {
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  WiFi.onEvent(WiFiEvent);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);
  connectToWifi();
}

void fun_envio_mqtt() {
  fun_saca();           ////veo si hay valores nuevos
  if (flag_vacio == 0)  ////si hay los envio
  {
    Serial.print("enviando");
    ////genero el string a enviar 1. 2.   3.   4.   5.   6.   7   8  9.       1.         2.        3.      4.       5.           6.   7.        8.        9
    snprintf(mqtt_payload, 150, "%u&%ld&%.2f&%.2f&%.2f&%.2f&%.2f&%u&%u", name_device, aux2.time, aux2.T1, aux2.H1, aux2.luz, aux2.G1, aux2.G2, aux2.oct, aux2.Alarma);  //random(10,50)
    aux2.time = 0;                                                                                                                                                      ///limpio valores
    aux2.T1 = 0;
    aux2.H1 = 0;
    aux2.luz = 0;
    aux2.G1 = 0;
    aux2.G2 = 0;
    aux2.oct = 0;
    aux2.Alarma = 0;

    Serial.print("Publish message: ");
    Serial.println(mqtt_payload);
    // Publishes Temperature and Humidity values
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB, 1, true, mqtt_payload);
  } else {
    Serial.println("no hay valores nuevos");
  }
}

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
}  ///////////////////////////////////////////////////
void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}  ///////////////////////////////////////////////////

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0);  // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}  ///////////////////////////////////////////////////
////////////////NO TOCAR FUNCIONES MQTT///////////////////////////////////
void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}  ///////////////////////////////////////////////////

////////////////NO TOCAR FUNCIONES MQTT///////////////////////////////////
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}
////////////////NO TOCAR FUNCIONES MQTT///////////////////////////////////
void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}  ///////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
/////////////Funcion que saca un valor de la estructura para enviar //////
///////////////////////////////////////////////////////////////////////
void fun_saca() {
  if (indice_saca != indice_entra) {
    aux2.time = datos_struct[indice_saca].time;
    aux2.T1 = datos_struct[indice_saca].T1;
    aux2.H1 = datos_struct[indice_saca].H1;
    aux2.luz = datos_struct[indice_saca].luz;
    aux2.G1 = datos_struct[indice_saca].G1;
    aux2.G2 = datos_struct[indice_saca].G2;
    aux2.oct = datos_struct[indice_saca].oct;
    aux2.Alarma = datos_struct[indice_saca].Alarma;

    flag_vacio = 0;

    Serial.println(indice_saca);
    if (indice_saca >= (valor_max_struct - 1)) {
      indice_saca = 0;
    } else {
      indice_saca++;
    }
    Serial.print("saco valores de la struct isaca:");
    Serial.println(indice_saca);
  } else {
    flag_vacio = 1;  ///// no hay datos
  }
  return;
}

//ingresa los valores a la cola struct
void fun_entra(void) {
  if (indice_entra >= valor_max_struct) {
    indice_entra = 0;  ///si llego al maximo de la cola se vuelve a cero
  }
  //////////// timestamp/////// consigo la hora
  Serial.print("> NTP Time:");
  timestamp = time(NULL);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;  //// si no puede conseguir la hora
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  ///////////////////////// fin de consigo la hora
  datos_struct[indice_entra].time = timestamp;
  datos_struct[indice_entra].T1 = lectemp;  /// leeo los datos //aca va la funcion de cada sensor
  datos_struct[indice_entra].H1 = lechum;   //// se puede pasar por un parametro valor entre 0 y 100
  datos_struct[indice_entra].luz = ldr;
  datos_struct[indice_entra].G1 = gas1;
  datos_struct[indice_entra].G2 = gas2;
  datos_struct[indice_entra].oct = 1;
  datos_struct[indice_entra].Alarma = 1;

  indice_entra++;
  Serial.print("saco valores de la struct ientra");
  Serial.println(indice_entra);
}



void setup() {
  Serial.begin(115200);
  Serial.println("PROGRAMA INICIADO");

  pinMode(BOT_PANTALLA, INPUT_PULLUP);
  pinMode(BOT_SELECT, INPUT_PULLUP);
  pinMode(BOT_SUMA, INPUT_PULLUP);
  pinMode(BOT_RESTA, INPUT_PULLUP);
  pinMode(BOT_SAVE, INPUT_PULLUP);

  pinMode(PIN_RELE, OUTPUT);
  digitalWrite(PIN_RELE, LOW);

  lcd.init();
  lcd.backlight();
  aht.begin();

  //WIFI
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  client.setCACert(TELEGRAM_CERTIFICATE_ROOT);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("-");
  }
  Serial.println("WiFi conectado");
  bot.sendMessage(CHAT_ID, "BOT iniciado", "");

  xTaskCreatePinnedToCore(
    Task1code,  //Nombre de la funcion
    "Task1",    //Nombre de la tarea
    10000,      //Limite de espacio de la tarea
    NULL,       //Parametro
    1,          //Prioridad de la tarea
    &Task1,     //A que tarea se refiere
    0           //A que nucleo del ESP32 se asigna la tarea
  );

  xTaskCreatePinnedToCore(
    Task2code,  //Nombre de la funcion
    "Task2",    //Nombre de la tarea
    10000,      //Limite de espacio de la tarea
    NULL,       //Parametro
    1,          //Prioridad de la tarea
    &Task2,     //A que tarea se refiere
    1           //A que nucleo del ESP32 se asigna la tarea
  );

  VU_eeprom.begin("valor-VU", false);
  umbral_gas1 = VU_eeprom.getFloat("ug1", 24.0);
  umbral_gas2 = VU_eeprom.getFloat("ug2", 24.0);
  umbral_temp = VU_eeprom.getFloat("ut", 24.0);
  umbral_hum = VU_eeprom.getFloat("uh", 24.0);
  umbral_ldr = VU_eeprom.getFloat("ul", 24.0);
  gmt_user = VU_eeprom.getInt("gmt_user", 3);

  //MQTT
  setupmqtt();
  //Setup de time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  gmt_offset = -gmt_user * 3600;
}

void loop() {

  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  lectemp = temp.temperature;
  lechum = humidity.relative_humidity;
  //ALERTA SUPERA AL UMBRAL
  if (lectemp >= umbral_temp) {
    digitalWrite(PIN_RELE, HIGH);
  } else {
    digitalWrite(PIN_RELE, LOW);
  }
}

void Task1code(void* pvParameters) {
  unsigned long ultimoChequeo = 0;
  const unsigned long intervalo = 1000;

  for (;;) {

    unsigned long ahora = millis();

    if (ahora - ultimoChequeo >= intervalo) {
      ultimoChequeo = ahora;

      //ACTUALIZAR SENSORES
      sensors_event_t humidity, temp;
      aht.getEvent(&humidity, &temp);

      float t = temp.temperature;
      float h = humidity.relative_humidity;

      int g1 = analogRead(MQA1);
      int g2 = analogRead(MQA2);
      int l = analogRead(LDR_PIN);

      if (g1 >= umbral_gas1 && checkeo_gas1 == 0) {
        checkeo_gas1 = 1;
        bot.sendMessage(CHAT_ID, "⚠️ SE SUPERÓ EL UMBRAL DE GAS 1");
      }

      if (g2 >= umbral_gas2 && checkeo_gas2 == 0) {
        checkeo_gas2 = 1;
        bot.sendMessage(CHAT_ID, "⚠️ SE SUPERÓ EL UMBRAL DE GAS 2");
      }

      if (t >= umbral_temp && checkeo_temp == 0) {
        checkeo_temp = 1;
        bot.sendMessage(CHAT_ID, "⚠️ SE SUPERÓ EL UMBRAL DE TEMPERATURA");
      }

      if (h >= umbral_hum && checkeo_hum == 0) {
        checkeo_hum = 1;
        bot.sendMessage(CHAT_ID, "⚠️ SE SUPERÓ EL UMBRAL DE HUMEDAD");
      }

      if (l >= umbral_ldr && checkeo_ldr == 0) {
        checkeo_ldr = 1;
        bot.sendMessage(CHAT_ID, "⚠️ SE SUPERÓ EL UMBRAL DE LUZ (LDR)");
      }

      if (g1 < umbral_gas1 && g2 < umbral_gas2 && t < umbral_temp && h < umbral_hum && l < umbral_ldr) {
        checkeo_gas1 = 0;
        checkeo_gas2 = 0;
        checkeo_temp = 0;
        checkeo_hum = 0;
        checkeo_ldr = 0;
      }
    }
  }
}

void Task2code(void* pvParameters) {
  for (;;) {
    MDE();
    now = millis();
    if (now - lastMeasure1 > interval_envio) {  ////envio el doble de lectura por si falla algun envio
      lastMeasure1 = now;                       /// cargo el valor actual de millis
      fun_envio_mqtt();                         ///envio los valores por mqtt
    }
    if (now - lastMeasure2 > interval_leeo) {
      lastMeasure2 = now;  /// cargo el valor actual de millis
      fun_entra();         ///ingreso los valores a la cola struct
    }
  }
}

void MDE() {

  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  ldr = map(analogRead(LDR_PIN), 0, 4095, 0, 100);
  gas1 = map(analogRead(MQA1), 0, 4095, 0, 100);
  gas2 = map(analogRead(MQA2), 0, 4095, 0, 100);

  switch (ESTADO) {

    case VAL_GAS1:
      Serial.println("VAL_GAS1");
      reglon1 = "VALOR GAS1: " + String(gas1);
      reglon2 = "UMBRAL: " + String(umbral_gas1);
      fun_lcd();

      if (botonESTADO(BOT_SUMA)) {
        umbral_gas1 = umbral_gas1 + 1;
      }
      if (botonESTADO(BOT_RESTA)) {
        umbral_gas1 = umbral_gas1 - 1;
      }
      if (digitalRead(BOT_SUMA) == LOW && digitalRead(BOT_RESTA) == LOW) {

        VU_eeprom.begin("valor-VU", false);
        VU_eeprom.putFloat("ug1", umbral_gas1);
        VU_eeprom.end();
        contando = millis();
        ESTADO = GUARDADO;
      }
      if (botonESTADO(BOT_SELECT)) {
        contando_select++;
        if (contando_select == 1) {
          ESTADO = VAL_GAS2;
        }
      }
      break;

    case VAL_GAS2:
      Serial.println("VAL_GAS2");
      reglon1 = "VALOR GAS2: " + String(gas2);
      reglon2 = "UMBRAL: " + String(umbral_gas2);
      fun_lcd();
      if (botonESTADO(BOT_SUMA)) {
        umbral_gas2 = umbral_gas2 + 2;
      }
      if (botonESTADO(BOT_RESTA)) {
        umbral_gas2 = umbral_gas2 - 2;
      }
      if (botonESTADO(BOT_SELECT)) {
        contando_select++;
        if (contando_select == 2) {
          ESTADO = VAL_TEMP;
        }
      }
      if (botonESTADO(BOT_PANTALLA)) {
        contando_select--;
        if (contando_select == 0) {
          ESTADO = VAL_GAS1;
        }
      }

      if (digitalRead(BOT_SUMA) == LOW && digitalRead(BOT_RESTA) == LOW) {

        VU_eeprom.begin("valor-VU", false);
        VU_eeprom.putFloat("ug2", umbral_gas2);
        VU_eeprom.end();

        contando = millis();
        ESTADO = GUARDADO;
      }
      break;

    case VAL_TEMP:
      Serial.println("VAL_TEMP");
      reglon1 = "VALOR TEMP: " + String(lectemp);
      reglon2 = "UMBRAL: " + String(umbral_temp);
      fun_lcd();
      if (botonESTADO(BOT_SUMA)) {
        umbral_temp = umbral_temp + 1;
      }
      if (botonESTADO(BOT_RESTA)) {
        umbral_temp = umbral_temp - 1;
      }
      if (botonESTADO(BOT_SELECT)) {
        contando_select++;
        if (contando_select == 3) {
          ESTADO = VAL_HUM;
        }
      }
      if (botonESTADO(BOT_PANTALLA)) {
        contando_select--;
        if (contando_select == 1) {
          ESTADO = VAL_GAS2;
        }
      }
      if (digitalRead(BOT_SUMA) == LOW && digitalRead(BOT_RESTA) == LOW) {

        VU_eeprom.begin("valor-VU", false);
        VU_eeprom.putFloat("ut", umbral_temp);
        VU_eeprom.end();

        contando = millis();
        ESTADO = GUARDADO;
      }
      break;

    case VAL_HUM:
      Serial.println("VAL_HUM");
      reglon1 = "VALOR HUM: " + String(lechum);
      reglon2 = "UMBRAL: " + String(umbral_hum);
      fun_lcd();
      if (botonESTADO(BOT_SUMA)) {
        umbral_hum = umbral_hum + 1;
      }
      if (botonESTADO(BOT_RESTA)) {
        umbral_hum = umbral_hum - 1;
      }
      if (botonESTADO(BOT_SELECT)) {
        contando_select++;
        if (contando_select == 4) {
          ESTADO = VAL_LDR;
        }
      }
      if (botonESTADO(BOT_PANTALLA)) {
        contando_select--;
        if (contando_select == 2) {
          ESTADO = VAL_TEMP;
        }
      }

      if (digitalRead(BOT_SUMA) == LOW && digitalRead(BOT_RESTA) == LOW) {

        VU_eeprom.begin("valor-VU", false);
        VU_eeprom.putFloat("uh", umbral_hum);
        VU_eeprom.end();

        contando = millis();
        ESTADO = GUARDADO;
      }
      break;

    case VAL_LDR:
      Serial.println("VAL_LDR");
      reglon1 = "VALOR LDR: " + String(ldr);
      reglon2 = "UMBRAL: " + String(umbral_ldr);
      fun_lcd();
      if (botonESTADO(BOT_SUMA)) {
        umbral_ldr = umbral_ldr + 1;
      }
      if (botonESTADO(BOT_RESTA)) {
        umbral_ldr = umbral_ldr - 1;
      }
      if (botonESTADO(BOT_SELECT)) {
        contando_select++;
        if (contando_select == 5) {
          ESTADO = VAL_GMT;
        }
      }
      if (botonESTADO(BOT_PANTALLA)) {
        contando_select--;
        if (contando_select == 3) {
          ESTADO = VAL_HUM;
        }
      }
      if (digitalRead(BOT_SUMA) == LOW && digitalRead(BOT_RESTA) == LOW) {

        VU_eeprom.begin("valor-VU", false);
        VU_eeprom.putFloat("ul", umbral_ldr);
        VU_eeprom.end();

        contando = millis();
        ESTADO = GUARDADO;
      }
      break;

      case VAL_GMT:
      reglon1 = "VALOR GMT: ";
      reglon2 = String(gmt_user);
      fun_lcd();

      if (botonESTADO(BOT_SUMA)) {
        if (gmt_user == 12) {
          gmt_user = -12;
        } else {
          gmt_user++;
        }
      }
      if (botonESTADO(BOT_RESTA)) {
        if (gmt_user == -12) {
          gmt_user = 12;
        } else {
          gmt_user--;
        }
      }

      if (botonESTADO(BOT_SELECT)) {
        contando_select++;
        if (contando_select == 6) {
          ESTADO = VAL_GAS1;
        }
      }

      if (botonESTADO(BOT_PANTALLA)) {
        contando_select--;
        if (contando_select == 4) {
          ESTADO = VAL_LDR;
        }
      }

      if (digitalRead(BOT_SUMA) == LOW && digitalRead(BOT_RESTA) == LOW) {

        VU_eeprom.begin("valor-VU", false);
        VU_eeprom.putInt("gmt_user", gmt_user);
        VU_eeprom.end();

        contando = millis();
        ESTADO = GUARDADO;
      }

      if(digitalRead(BOT_PANTALLA) == LOW) {
        ESTADO = VAL_GAS1;
      }
      break;

    case GUARDADO:
      Serial.println("GUARDADO");
      reglon1 = "CAMBIOS";
      reglon2 = "GUARDADOS";
      fun_lcd();
      if ((millis() - contando) >= 2000) {
        contando = 0;
        contando_select = 0;
        ESTADO = VAL_GAS1;
      }
      break;
  }
}

bool botonESTADO(int pin) {
  static unsigned long ultimoTiempo[40] = { 0 };
  static bool ultimoEstado[40] = { HIGH };

  bool estadoActual = digitalRead(pin);
  unsigned long ahora = millis();

  if (ahora - ultimoTiempo[pin] < 50) return false;

  if (ultimoEstado[pin] == HIGH && estadoActual == LOW) {
    ultimoEstado[pin] = estadoActual;
    ultimoTiempo[pin] = ahora;
    return true;
  }

  if (estadoActual != ultimoEstado[pin]) {
    ultimoEstado[pin] = estadoActual;
    ultimoTiempo[pin] = ahora;
  }

  return false;
}

void fun_lcd() {
  static unsigned long ultimoRefresco = 0;

  // Actualiza la pantalla cada 200 ms para evitar parpadeo
  if (millis() - ultimoRefresco >= 200) {
    ultimoRefresco = millis();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(reglon1);
    lcd.setCursor(0, 1);
    lcd.print(reglon2);
  }
}
