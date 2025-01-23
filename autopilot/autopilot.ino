#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <math.h>
#include "ARA_ESP.h"
#include "DxlMaster2.h"


#define EMULATE


const char* ssid = "ESP_AUTOPILOT";
const char* password = "12345678";

AsyncWebServer server(80);

// Хранение точек маршрута
struct Waypoint {
    uint8_t id;
    float x;
    float y;
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
struct Point {
  float x;
  float y;
};

#define MAX_X 2500
#define MAX_Y 3000
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
uint32_t measure_count[4];
uint32_t measure_time[4];
Vector4 p[4];

float vectorLength(Vector3 v) {
  return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

float angle;
float pitch;
float roll;

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
    if (request->hasParam("x") && request->hasParam("y")) {
        float x = request->getParam("x")->value().toFloat();
        float y = request->getParam("y")->value().toFloat();

        waypoints.push_back({ waypointCounter++, x, y });

        request->send(200, "application/json", "{\"status\":\"success\"}");
    } else {
        request->send(400, "application/json", "{\"status\":\"error\", \"message\":\"Missing parameters\"}");
    }
}

// Обработчик для удаления точки
void handleDeleteWaypoint(AsyncWebServerRequest *request) {
    if (request->hasParam("id")) {
        int id = request->getParam("id")->value().toInt();
        waypoints.erase(std::remove_if(waypoints.begin(), waypoints.end(), [id](const Waypoint& wp) {
            return wp.id == id;
        }), waypoints.end());

        request->send(200, "application/json", "{\"status\":\"success\"}");
    } else {
        request->send(400, "application/json", "{\"status\":\"error\", \"message\":\"Missing parameters\"}");
    }
}

// Обработчик для получения списка точек
void handleGetWaypoints(AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(1024);

    JsonArray arr = doc.to<JsonArray>();
    for (const auto& wp : waypoints) {
        JsonObject obj = arr.createNestedObject();
        obj["id"] = wp.id;
        obj["x"] = wp.x;
        obj["y"] = wp.y;
    }

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

// Обработчик для изменения размера карты
void handleSetMapSize(AsyncWebServerRequest *request) {
    if (request->hasParam("width") && request->hasParam("height")) {
        mapWidth = request->getParam("width")->value().toFloat();
        mapHeight = request->getParam("height")->value().toFloat();

        request->send(200, "application/json", "{\"status\":\"success\"}");
    } else {
        request->send(400, "application/json", "{\"status\":\"error\", \"message\":\"Missing parameters\"}");
    }
}

void handleSetWaypoint(AsyncWebServerRequest *request) {
    if (request->hasParam("id") && request->hasParam("x") && request->hasParam("y")) {
        int id = request->getParam("id")->value().toInt();
        float x = request->getParam("x")->value().toFloat();
        float y = request->getParam("y")->value().toFloat();

        for (auto& wp : waypoints) {
            if (wp.id == id) {
                wp.x = x;
                wp.y = y;
                break;
            }
        }

        request->send(200, "application/json", "{\"status\":\"success\"}");
    } else {
        request->send(400, "application/json", "{\"status\":\"error\", \"message\":\"Missing parameters\"}");
    }
}

// Обработчик для получения позиции дрона
void handleGetDronePosition(AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(1024);
    doc["x"] = position.x;
    doc["y"] = position.y;
    doc["angle"] = angle; // расчет угла
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}


void handleGetChannels(AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(1024);
    doc["Roll"] = roll;   // Пример
    doc["Pitch"] = pitch;
    doc["Throttle"] = 0;
    doc["Yaw"] = 0;
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}



// HTML-страница
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset = "utf-8">
    <title>ESP AUTOPILOT</title>
    <style>
        #map {
            width: 60%;
            height: 500px;
            float: left;
            border: 1px solid black;
            position: relative;
        }
        #sidebar {
            width: 30%;
            float: right;
        }
        .waypoint {
            margin: 5px 0;
        }
        .map-point {
            width: 10px;
            height: 10px;
            background: red;
            position: absolute;
            border-radius: 50%;
        }
        .drone {
            position: absolute;
            border-radius: 50%;
            width: 20px;
            height: 20px;
            background: blue;
            background-size: cover;
        }
        .direction {
            position: absolute;
            background: green;
            height: 2px;
            transform-origin: 0% 50%;
        }

    </style>
    <script>
        let waypoints = [];
        let mapWidth = 3000;
        let mapHeight = 3000;
        let drone = { x: 50, y: 50 };

        async function updateDronePosition() {
            const response = await fetch('/get_drone_position');
            const data = await response.json();
            drone = { x: data.x, y: data.y, angle: data.angle };
            const droneDiv = document.getElementById('drone_data');
            droneDiv.innerHTML = ''; // Очистка перед обновлением

            const xDiv = document.createElement('div');
            xDiv.innerText = `Pos X: ${data.x}`;
            droneDiv.appendChild(xDiv);

            const yDiv = document.createElement('div');
            yDiv.innerText = `Pos Y: ${data.y}`;
            droneDiv.appendChild(yDiv);

            const angleDiv = document.createElement('div');
            angleDiv.innerText = `Angle: ${data.angle}`;
            droneDiv.appendChild(angleDiv);

            renderDrone();
        }

        function renderDrone() {
            const map = document.getElementById('map');
            let droneElem = document.querySelector('.drone');
            if (!droneElem) {
                droneElem = document.createElement('div');
                droneElem.className = 'drone';
                map.appendChild(droneElem);
            }
            droneElem.style.left = `${(drone.x / mapWidth) * 100}%`;
            droneElem.style.top = `${(drone.y / mapHeight) * 100}%`;

            // Угол и вектор
            const angle = drone.angle || 0; // угол дрона, полученный из данных
            const vectorElem = document.querySelector('.direction');
            if (!vectorElem) {
                const vec = document.createElement('div');
                vec.className = 'direction';
                vec.style.position = 'absolute';
                vec.style.width = '60px';
                vec.style.height = '2px';
                vec.style.background = 'green';
                vec.style.transformOrigin = 'left center';
                map.appendChild(vec);
            }
            vectorElem.style.left = `${(drone.x / mapWidth) * 100}%`;
            vectorElem.style.top = `${(drone.y / mapHeight) * 100}%`;
            vectorElem.style.transform = `rotate(${angle}deg)`;
        }


        setInterval(updateDronePosition, 100); // Каждые 1 секунду обновляем
        async function updateChannels() {
            const response = await fetch('/get_channels');
            const channels = await response.json();
            const channelsDiv = document.getElementById('channels');
            channelsDiv.innerHTML = ''; // Очистка перед обновлением
            for (const [channel, value] of Object.entries(channels)) {
                const div = document.createElement('div');
                div.innerText = `Канал ${channel}: ${value}`;
                channelsDiv.appendChild(div);
            }
        }
        setInterval(updateChannels, 100); // Обновление каждые 250 мс
        async function addWaypoint(x, y) {
            const response = await fetch(`/add?x=${x}&y=${y}`, { method: 'POST' });
            const result = await response.json();
            if (result.status === 'success') {
                fetchWaypoints();
            }
        }

        async function deleteWaypoint(id) {
            const response = await fetch(`/delete?id=${id}`, { method: 'POST' });
            const result = await response.json();
            if (result.status === 'success') {
                fetchWaypoints();
            }
        }

        async function setMapSize(width, height) {
            const response = await fetch(`/set_map_size?width=${width}&height=${height}`, { method: 'POST' });
            const result = await response.json();
            if (result.status === 'success') {
                mapWidth = width;
                mapHeight = height;
                alert('Map size updated');
                fetchWaypoints();
            }
        }

        async function fetchWaypoints() {
            const response = await fetch('/waypoints');
            waypoints = await response.json();
            renderWaypoints();
        }

        function renderWaypoints() {
            const sidebar = document.getElementById('waypoints');
            const map = document.getElementById('map');
            sidebar.innerHTML = '';
            map.innerHTML = '';
            waypoints.forEach(wp => {
                // Отображение в списке
                const div = document.createElement('div');
                div.className = 'waypoint';
                // Отображение дрона
                const droneElem = document.createElement('div');
                droneElem.className = 'drone';
                droneElem.style.left = `${(drone.x / mapWidth) * 100}%`;
                droneElem.style.top = `${(drone.y / mapHeight) * 100}%`;
                map.appendChild(droneElem);

                const inputX = document.createElement('input');
                inputX.type = 'number';
                inputX.value = wp.x;
                inputX.style.marginRight = '5px';

                const inputY = document.createElement('input');
                inputY.type = 'number';
                inputY.value = wp.y;
                inputY.style.marginRight = '5px';

                const applyButton = document.createElement('button');
                applyButton.innerText = 'Применить';
                applyButton.onclick = () => updateWaypoint(wp.id, parseFloat(inputX.value), parseFloat(inputY.value));

                const deleteButton = document.createElement('button');
                deleteButton.innerText = 'Удалить';
                deleteButton.onclick = () => deleteWaypoint(wp.id);

                div.appendChild(inputX);
                div.appendChild(inputY);
                div.appendChild(applyButton);
                div.appendChild(deleteButton);
                sidebar.appendChild(div);

                // Отображение на карте
                const point = document.createElement('div');
                point.className = 'map-point';
                point.style.left = `${(wp.x / mapWidth) * 100}%`;
                point.style.top = `${(wp.y / mapHeight) * 100}%`;
                map.appendChild(point);
            });
        }

        async function updateWaypoint(id, x, y) {
            const response = await fetch(`/set_waypoint?id=${id}&x=${x}&y=${y}`, { method: 'POST' });
            const result = await response.json();
            if (result.status === 'success') {
                fetchWaypoints();
            }
        }

        document.addEventListener('DOMContentLoaded', () => {
            const map = document.getElementById('map');
            map.addEventListener('click', (e) => {
                const rect = map.getBoundingClientRect();
                const x = ((e.clientX - rect.left) / rect.width) * mapWidth;
                const y = ((e.clientY - rect.top) / rect.height) * mapHeight;
                addWaypoint(x.toFixed(2), y.toFixed(2));
            });

            const setMapButton = document.getElementById('setMapSize');
            setMapButton.addEventListener('click', () => {
                const width = document.getElementById('mapWidth').value;
                const height = document.getElementById('mapHeight').value;
                setMapSize(width, height);
            });

            fetchWaypoints();
        });
    </script>
</head>
<body>
    <h1>Автономный полет ESP</h1>
    <div>

    </div>
    <div id="map"></div>
<div id="sidebar">
    <h3>Управление</h3>
    <h4>Размер карты</h4>
        <label>Ширина: <input type="number" id="mapWidth" value="3000" style="margin-right: 10px;"></label>
        <label>Высота: <input type="number" id="mapHeight" value="3000" style="margin-right: 10px;"></label>
        <button id="setMapSize">Применить</button>
    <h4>Точки маршрута</h4>
    <div id="waypoints"></div>
    <h4>Точки маршрута</h4>
    <div id="waypoints"></div>
    <h4>Данные каналов</h4>
    <div id="channels"></div>
    <h4>Данные дрона</h4>
    <div id="drone_data"></div>
</div>
</body>
</html>
)rawliteral";

uint64_t timer = 0;

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, 2, 4);
    esp.begin(Serial2);
    // Подключение к Wi-Fi
    WiFi.softAP(ssid, password);

    IPAddress IP = WiFi.softAPIP();
    Serial.println("Точка доступа запущена:");
    Serial.print("SSID: ");
    Serial.println(ssid);
    Serial.print("Password: ");
    Serial.println(password);
    Serial.print("IP Address: ");
    Serial.println(IP);

    // Обработчики маршрутов
    server.on("/add", HTTP_POST, handleAddWaypoint);
    server.on("/delete", HTTP_POST, handleDeleteWaypoint);
    server.on("/waypoints", HTTP_GET, handleGetWaypoints);
    server.on("/set_map_size", HTTP_POST, handleSetMapSize);
    server.on("/set_waypoint", HTTP_POST, handleSetWaypoint);
    server.on("/get_drone_position", HTTP_GET, handleGetDronePosition);
    server.on("/get_channels", HTTP_GET, handleGetChannels);

    // Главная страница
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/html", index_html);
    });

    server.begin();

    delay(1000);  // Ждать стабилизации системы

    p[0] = {0    , 0    , MAX_Z, 3000}; // Положение передатчиков по углам
    p[1] = {MAX_X, 0    , MAX_Z, 3000};
    p[2] = {0    , MAX_Y, MAX_Z, 3000};
    p[3] = {MAX_X, MAX_Y, MAX_Z, 3000};
    position.x = 1500;
    position.y = 1000;
}
//roll < 1500 - влево 
//pitch < 1500 - назад



// Заданные точки
Point current_position = {0, 0}; // Начальная позиция дрона
Point target_position = {5, 5}; // Целевая позиция

// Параметры управления
uint16_t speed = 100;        // Скорость движения (м/с)
float tolerance = 0.1;    // Допустимая погрешность в метрах
float update_interval = 0.1; // Интервал обновления (с)
uint8_t counter_wp_checked = 0;\
uint8_t msp_overwrite = 0;
uint8_t alt_hold_on = 0;
uint8_t autopilot_on = 0;
// Функция для вычисления угла между двумя точками
float calculate_angle(Point from, Point to) {
  return atan2(to.y - from.y, to.x - from.x) * 180.0 / M_PI;
}

void check_way_point(Vector3 drone_pos, Waypoint* wp)
{
  uint16_t len_x = abs(drone_pos.x - wp->x);
  uint16_t len_y = abs(drone_pos.y - wp->y);
  if (len_x < 100 && len_y < 100)
  {
    wp->checked = 1;
    counter_wp_checked++;
    esp.pitch(0);
    esp.roll(0);
    esp.throttle(0.3);
    delay(1000);
  } 
}

// Функция для обновления положения дрона
void update_position(Point current_position, Point target_position) {
  angle = calculate_angle(current_position, target_position);
  
  // Рассчитываем наклоны (roll и pitch) для движения к цели
  pitch = sin(angle * M_PI / 180.0)*0.3;
  roll = cos(angle * M_PI / 180.0)*0.3;

  #ifdef EMULATE
    position.x += 50*roll;
    position.y += 50*pitch; 
  #endif
  // Управляем дроном
  esp.pitch(pitch);
  esp.roll(roll);
  esp.throttle(0.5); // Поддержание скорости (примерная мощность)
  Serial.printf("%d %d\n", pitch, roll);
  // Выводим информацию в Serial
  Serial.print("Current Position: X=");
  Serial.print(current_position.x);
  Serial.print(", Y=");
  Serial.println(current_position.y);
}


uint8_t msp_failed_counter = 0;
void loop() {
  uint64_t diff_timer = millis() - timer;
  if (diff_timer > 200)
  {
    timer = millis();
    uint16_t aux1 = esp.get_channel(6); //alt hold
    uint16_t aux2 = esp.get_channel(8); //msp overwrite
    // if (aux1 == 0)
    // {
    //   msp_failed_counter++;
    // }
    // if (msp_failed_counter >= 10)
    // {
    //   msp_failed_counter = 0;
    //   esp.begin(Serial2);
    // }
    // Serial.printf("ch6 = %d, ch8 = %d\n", aux1, aux2);
    alt_hold_on = aux1 >=  1500 ? 1 : 0;
    msp_overwrite = aux2 > 1500 ? 1 : 0;
    
    autopilot_on = alt_hold_on && msp_overwrite;
    #ifdef EMULATE
      if (counter_wp_checked >= waypointCounter)
      {

      }
      else if (!waypoints.empty()) {
        Waypoint* current_wp = &waypoints[counter_wp_checked];
        check_way_point(position, current_wp);
        update_position({position.x, position.y}, {waypoints[counter_wp_checked].x, waypoints[counter_wp_checked].y});
        // check_way_point(position, &(waypoints[counter_wp_checked]));
      }
    #endif
    if (autopilot_on == 1)
    {
      if (counter_wp_checked >= waypointCounter)
      {
        esp.throttle(0.2);
        delay(500);
        esp.throttle(0);
        for(;;){}
      }
      else if (!waypoints.empty()) {
        Waypoint* current_wp = &waypoints[counter_wp_checked];
        check_way_point(position, current_wp);
        update_position({position.x, position.y}, {waypoints[counter_wp_checked].x, waypoints[counter_wp_checked].y});
        // check_way_point(position, &(waypoints[counter_wp_checked]));
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
      if (num1 < 4)
      {     
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
        Serial.printf("pos = %f %f %f\n", position.x, position.y, position.z);
      }
    }
  }
    
}
