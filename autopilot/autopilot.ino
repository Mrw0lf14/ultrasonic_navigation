#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <math.h>
#include "ARA_ESP.h"
#include "DxlMaster2.h"
#include <SPIFFS.h>
#include <deque>
#include <esp_wifi.h>
#include <string.h>

const int FILTER_SIZE = 5; // Размер окна фильтра
std::deque<float> x_history, y_history; // Очереди для хранения истории значений

#define EMULATE

char ssid[32];
const char* password = "12345678";

AsyncWebServer server(80);

enum WorkStatus {
  STATUS_WAIT_BASE = 0,
  STATUS_MANUAL,
  STATUS_AUTOPILOT,
  STATUS_MISSION_END,
};
uint8_t status_id = 0;

struct WorkState {
  uint8_t state_base;
  uint8_t state_channels;
  uint8_t state_autopilot;
  uint8_t state_wp_num;
};
WorkState states = {};

String status_msg [] = {"Ждем базу", "Ручной режим", "Автономный режим", "Миссия завершена"};
// Хранение точек маршрута
struct Waypoint {
    uint8_t id;
    float x;
    float y;
    float z;           //throttle
    uint8_t checked = 0;
};
std::vector<Waypoint> waypoints;
int waypointCounter = 0;
float mapWidth = 100.0;
float mapHeight = 100.0;

struct Vector4 {
  float x;
  float y;
  float z;
  float r;
};

struct Vector3 {
  float x;
  float y;
  float z;
};

// Структура для представления точки
struct Vector2 {
  float x;
  float y;
};

// Параметры управления
uint8_t counter_wp_checked = 0;

uint16_t MAX_X = 2500;
uint16_t MAX_Y = 3000;
#define MAX_Z 2400
#define ka 1
#define kb 0

#define INAV_SERIAL Serial2
#define DXL_SERIAL Serial

static unsigned long previousMillis = 0;
const long interval = 200;
static uint8_t ack_en = 0;
static uint8_t count = 0;
static uint8_t dlay = 0;

Vector3 position;
Vector4 p[4];

// Функция обновления фильтра и получения сглаженного значения
float updateFilter(std::deque<float>& history, float newValue) {
    history.push_back(newValue);
    if (history.size() > FILTER_SIZE) {
        history.pop_front();
    }

    float sum = 0;
    for (float val : history) {
        sum += val;
    }
    return sum / history.size();
}

float vectorLength(Vector3 v) {
  return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

float angle;
float pitch;
float roll;
float throttle;

Vector4 intersectionLength(Vector4 c1, Vector4 c2, uint8_t zflag = 0) {
  Vector3 distance = {c2.x - c1.x, c2.y - c1.y, c2.z - c1.z};
  float d = vectorLength(distance);

  float x,y,z;

  if (d > c1.r + c2.r || d < abs(c1.r - c2.r)) {
    Vector3 r1, r2;
    float k = c1.r/d;
    r1.x = c1.x + (float) k * (c2.x-c1.x); 
    r1.y = c1.y + (float) k * (c2.y-c1.y);
    r1.z = c1.z + (float) k * (c2.z-c1.z);
    k = c2.r/d;
    r2.x = c2.x + k * (c1.x-c2.x); 
    r2.y = c2.y + k * (c1.y-c2.y); 
    r2.z = c2.z + k * (c1.z-c2.z);

    return {(r1.x+r2.x)/2,(r1.y+r2.y)/2,(r1.z+r2.z)/2, 0};
  }

  float a = (c1.r * c1.r - c2.r * c2.r + d * d) / (2 * d);
  float h = sqrt(c1.r * c1.r - a * a);

  Vector3 point = {c1.x + a * (c2.x - c1.x) / d, c1.y + a * (c2.y - c1.y) / d, c1.z + a * (c2.z - c1.z) / d};

  Vector3 intersection1 = {point.x + h * (c2.y - c1.y) / d, point.y - h * (c2.x - c1.x) / d, point.z - h};
  Vector3 intersection2 = {point.x - h * (c2.y - c1.y) / d, point.y + h * (c2.x - c1.x) / d, point.z - h};

  x = (intersection1.x + intersection2.x) / 2;
  y = (intersection1.y + intersection2.y) / 2;

  z = MAX_Z;
  if (zflag == 1)
  {
      z = intersection1.z;
  }


  Vector4 tmp = {x, y, z, h};
  return tmp;
}

// Обработчик для добавления точки
void handleAddWaypoint(AsyncWebServerRequest *request) {
    if (request->hasParam("x") && request->hasParam("y") && request->hasParam("z")) {
        float x = request->getParam("x")->value().toFloat();
        float y = request->getParam("y")->value().toFloat();
        float z = (float)(request->getParam("z")->value().toInt() - 1000) / 1000;
        Serial.printf("add z %f %d\n", z, request->getParam("z")->value().toInt());

        waypointCounter++;

        // Найти максимальный id
        int new_id = 1; // Если список пуст
        if (!waypoints.empty()) {
            new_id = (*std::max_element(waypoints.begin(), waypoints.end(), 
                [](const Waypoint& a, const Waypoint& b) { return a.id < b.id; })).id + 1;
        }
        waypoints.push_back({new_id , x, y, z});

        request->send(200, "application/json", "{\"status\":\"success\"}");
    } else {
        request->send(400, "application/json", "{\"status\":\"error\", \"message\":\"Missing parameters\"}");
    }
}

// Обработчик для удаления точки
void handleDeleteWaypoint(AsyncWebServerRequest *request) {
    if (request->hasParam("id")) {  
        int id = request->getParam("id")->value().toInt();
        Serial.printf("del %d\n", id);

        waypointCounter--;
        if (id-1 < counter_wp_checked)
        {
          counter_wp_checked--;
        }
        // Удаление указанного waypoint
        waypoints.erase(std::remove_if(waypoints.begin(), waypoints.end(), [id](const Waypoint& wp) {
            return wp.id == id;
        }), waypoints.end());

        // Формирование списка оставшихся id
        String remainingIds = "[";
        for (size_t i = 0; i < waypoints.size(); i++) {
            remainingIds += String(waypoints[i].id);
            if (i < waypoints.size() - 1) {
                remainingIds += ",";
            }
            waypoints[i].id = i+1;
        }
        remainingIds += "]";

        // Ответ клиенту
        String response = "{\"status\":\"success\", \"remaining_ids\":" + remainingIds + "}";
        request->send(200, "application/json", response);

        // Вывод оставшихся точек в Serial
        Serial.print("Remaining waypoint IDs: ");
        Serial.println(remainingIds);
    } else {
        request->send(400, "application/json", "{\"status\":\"error\", \"message\":\"Missing parameters\"}");
    }
}


void handleSetWaypoint(AsyncWebServerRequest *request) {
    if (request->hasParam("id") && request->hasParam("x") && request->hasParam("y") && request->hasParam("z")) {
        int id = request->getParam("id")->value().toInt();
        float x = request->getParam("x")->value().toFloat();
        float y = request->getParam("y")->value().toFloat();
        uint16_t z = request->getParam("z")->value().toInt();

        for (auto& wp : waypoints) {
            if (wp.id == id) {
                wp.x = x;
                wp.y = y;
                wp.z = (float)(z-1000)/1000;
                Serial.printf("p%d %d ", id, z);
                Serial.println(wp.z);
                break;
            }
        }

        request->send(200, "application/json", "{\"status\":\"success\"}");
    } else {
        request->send(400, "application/json", "{\"status\":\"error\", \"message\":\"Missing parameters\"}");
    }
}
// Обработчик для получения списка точек
void handleGetWaypoints(AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(1024);
    // Serial.println("Get WP");
    JsonArray arr = doc.to<JsonArray>();
    for (const auto& wp : waypoints) {
        JsonObject obj = arr.createNestedObject();
        obj["x"] = wp.x;
        obj["y"] = wp.y;
        obj["z"] = 1000 + wp.z*1000;
        obj["id"] = wp.id;
        obj["checked"] = wp.checked;  // Добавляем статус
    }

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

// Обработчик для изменения размера карты
void handleSetMapSize(AsyncWebServerRequest *request) {
    if (request->hasParam("width") && request->hasParam("height")) 
    {
      Serial.println("Set MapSize");
      mapWidth = request->getParam("width")->value().toFloat();
      mapHeight = request->getParam("height")->value().toFloat();
      MAX_X = mapWidth;
      MAX_Y = mapHeight;
      request->send(200, "application/json", "{\"status\":\"success\"}");
    } 
    else 
    {
      request->send(400, "application/json", "{\"status\":\"error\", \"message\":\"Missing parameters\"}");
    }
}

void handleGetMapSize(AsyncWebServerRequest *request) {
    // Serial.println("Get MapSize");
    DynamicJsonDocument doc(1024);
    doc["width"] = MAX_X;
    doc["height"] = MAX_Y;
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

// Обработчик для получения позиции дрона с фильтрацией
void handleGetDronePosition(AsyncWebServerRequest *request) {
    float fake_x = updateFilter(x_history, position.x);
    float fake_y = updateFilter(y_history, position.y);

    DynamicJsonDocument doc(1024);
    doc["x"] = fake_x;
    doc["y"] = fake_y;
    doc["angle"] = -angle - M_PI/2; // расчет угла

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}



void handleGetChannels(AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(1024);
    doc["Roll"] = esp.ROLL;   // Пример
    doc["Pitch"] = esp.PITCH;
    doc["Throttle"] = esp.THROTTLE;
    doc["Yaw"] = esp.YAW;
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

void handleRoot(AsyncWebServerRequest *request) {
    Serial.println("handleRoot вызван");
    if (!SPIFFS.exists("/index.html")) {
        Serial.println("index.html не найден в SPIFFS!");
        request->send(404, "text/plain", "File Not Found");
        return;
    }
    
    request->send(SPIFFS, "/index.html");
}

void handleGetStatus(AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(1024);
    doc["status"] = status_msg[status_id];
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

// char* readMacAddress()
// {
//   uint8_t baseMac[6];
//   esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
//   if (ret == ESP_OK) {
//     Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
//                   baseMac[0], baseMac[1], baseMac[2],
//                   baseMac[3], baseMac[4], baseMac[5]);
//   } else {
//     Serial.println("Failed to read MAC address");
//   }
//   return (char*) baseMac;
// }

void getUniqName(uint8_t len)
{
  uint64_t chip_id = ESP.getEfuseMac();   /* Чтение ID микроконтроллера */
  uint32_t mask = (1 << (len * 4)) - 1;
  chip_id >>= 24;   /* Формировние постфикса для имени сети из ID микроконтроллера */
  snprintf(ssid, sizeof(ssid), "ESP-US-NAV-%0*X", len,  (uint32_t)(chip_id & mask));
}

uint64_t timer = 0;
void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, 2, 4);
    esp.begin(Serial2);
    getUniqName(6);
    WiFi.softAP(ssid, password);

    if (!SPIFFS.begin(true)) {
      Serial.println("An Error has occurred while mounting SPIFFS");
      return;
    }

    IPAddress IP = WiFi.softAPIP();
    Serial.println("Точка доступа запущена:");
    Serial.print("SSID: ");
    Serial.println(ssid);
    Serial.print("Password: ");
    Serial.println(password);
    Serial.print("IP Address: ");
    Serial.println(IP);

    // Обработчики маршрутов
    server.on("/", HTTP_GET, handleRoot);
    server.on("/waypoint/add", HTTP_POST, handleAddWaypoint);
    server.on("/waypoint/delete", HTTP_POST, handleDeleteWaypoint);
    server.on("/waypoint/set", HTTP_POST, handleSetWaypoint);
    server.on("/waypoints", HTTP_GET, handleGetWaypoints);

    server.on("/map/size", HTTP_POST, handleSetMapSize);
    server.on("/map/size", HTTP_GET, handleGetMapSize);

    server.on("/drone/position", HTTP_GET, handleGetDronePosition);
    server.on("/channels", HTTP_GET, handleGetChannels);
    server.on("/status", HTTP_GET, handleGetStatus);
    server.begin();

    delay(1000);  // Ждать стабилизации системы

    p[0] = {0    , 0    , MAX_Z, 3000}; // Положение передатчиков по углам
    p[1] = {MAX_X, 0    , MAX_Z, 3000};
    p[2] = {0    , MAX_Y, MAX_Z, 3000};
    p[3] = {MAX_X, MAX_Y, MAX_Z, 3000};
    position.x = 1500;
    position.y = 1000;
}

// Функция для вычисления угла между двумя точками
float calculate_angle(Vector2 from, Vector2 to) {
  return atan2(to.y - from.y, to.x - from.x);
}

void check_way_point(Vector3 drone_pos, Waypoint* wp)
{
  uint16_t len_x = abs(drone_pos.x - wp->x);
  uint16_t len_y = abs(drone_pos.y - wp->y);
  if (len_x < 80 && len_y < 80)
  {
    wp->checked = 1;
    counter_wp_checked++;
    esp.pitch(0);
    esp.roll(0);
    delay(1000);
  } 
}

// Функция для обновления положения дрона
void update_position(Vector2 current_position, Vector3 target_position) {
  angle = calculate_angle(current_position, {target_position.x, target_position.y});
  
  // Рассчитываем наклоны (roll и pitch) для движения к цели
  pitch = sin(angle)*0.3;
  roll = cos(angle)*0.3;
  throttle = target_position.z;
  //проверка на границу с отступом
  float padding = 400;
  if ((position.x < padding) || (position.x > MAX_X - padding))
  {
    roll = -roll;
  } 
  if ((position.y < padding) || (position.y > MAX_Y - padding))
  {
    pitch = -pitch;
  }
  // Управляем дроном
#ifdef EMULATE
    position.x += 100*roll;
    position.y += 100*pitch; 
#endif
  esp.pitch(pitch);
  esp.roll(roll);
  esp.throttle(throttle); // Поддержание скорости (примерная мощность)
}


uint8_t msp_failed_counter = 0;
void loop() {
  uint64_t diff_timer = millis() - timer;
  if (diff_timer > 100)
  {
    if (states.state_base == 0)
    {
      status_id = STATUS_WAIT_BASE;
    }
    else if (states.state_channels == 0)
    {
      status_id = STATUS_MANUAL;
    }
    if (states.state_autopilot == 1)
    {
      status_id = STATUS_AUTOPILOT;
    }
    if (states.state_autopilot == 2)
    {
      status_id = STATUS_MISSION_END;
    }
    timer = millis();
    noInterrupts();
    uint16_t aux1 = esp.get_channel(6); //alt hold
    uint16_t aux2 = esp.get_channel(8); //msp overwrite
    interrupts();
#ifndef EMULATE
    if (aux1 == 0 && aux2 == 0)
    {
      msp_failed_counter++;
    }
    else 
    {
      msp_failed_counter = 0;
    }
    if (msp_failed_counter > 10)
    {
      // ESP.restart(); 
    }
#endif
    Serial.printf("ch6 = %d, ch8 = %d\n", aux1, aux2);
    uint8_t alt_hold_on = aux1 >=  1500 ? 1 : 0;
    uint8_t msp_overwrite = aux2 > 1500 ? 1 : 0;
    
    states.state_channels = alt_hold_on && msp_overwrite;
    if (states.state_channels == 0 || states.state_base == 0)
    {
      states.state_autopilot = 0;
    }
#ifdef EMULATE
      if (counter_wp_checked >= waypointCounter)
      {
        states.state_autopilot = 2;
      }
      else if (!waypoints.empty() && states.state_autopilot != 2) {
        Waypoint* current_wp = &waypoints[counter_wp_checked];
        check_way_point(position, current_wp);
        update_position({position.x, position.y}, {waypoints[counter_wp_checked].x, waypoints[counter_wp_checked].y, waypoints[counter_wp_checked].z});
        states.state_autopilot = 1;
      }
#endif
    if (states.state_channels == 1 && states.state_base == 1)
    {
      if (counter_wp_checked >= waypointCounter)
      {
        states.state_autopilot = 2;
        throttle = 0.2;
        roll = 0;
        pitch = 0;
        esp.throttle(throttle);
        esp.roll(roll);
        esp.pitch(pitch);
        delay(300);
        throttle = 0;
        esp.throttle(throttle);
        for(;;){}
      }
      else if (!waypoints.empty() && states.state_autopilot != 2) {
        states.state_autopilot = 1;
        Waypoint* current_wp = &waypoints[counter_wp_checked];
        check_way_point(position, current_wp);
        update_position({position.x, position.y}, {waypoints[counter_wp_checked].x, waypoints[counter_wp_checked].y, waypoints[counter_wp_checked].z});
      }
    }
  }
  if (DXL_SERIAL.available())
  {
    char head[4];
    head[0] = DXL_SERIAL.read();
    head[1] = DXL_SERIAL.read();
    head[2] = DXL_SERIAL.read();
    head[3] = DXL_SERIAL.read();
    if (strncmp(head, "DATA", 4) == 0)
    {
      String packet = DXL_SERIAL.readStringUntil('\n');
      int num1, num2, num3;
      sscanf(packet.c_str(), " %d %d %d", &num1, &num2, &num3);
      Serial.printf("%d %d %d\n\r", num1, num2, num3);
      if (num1 < 4)
      {
        states.state_base = 1;     
        p[num1].r = num2*ka + kb;

        Vector4 r[4];
        r[0] = intersectionLength(p[0], p[1], 0);
        r[1] = intersectionLength(p[0], p[2], 0);
        r[2] = intersectionLength(p[2], p[3], 0);
        r[3] = intersectionLength(p[1], p[3], 0);

        position.x = (r[0].x + r[2].x)/2;
        position.y = (r[1].y + r[3].y)/2;
        Vector4 rcr[2];
        rcr[0] = intersectionLength(r[0], r[2], 1);
        rcr[1] = intersectionLength(r[1], r[3], 1);
        position.z = (rcr[0].z + rcr[1].z)/2;
        position.x = constrain(position.x, 0, MAX_X);
        position.y = constrain(position.y, 0, MAX_Y);
        position.z = constrain(position.z, 0, MAX_Z);
        // Serial.printf("pos = %f %f %f\n", position.x, position.y, position.z);
      }
    }
  }
}
