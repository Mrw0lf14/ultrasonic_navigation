// Основной скрипт для работы веб-сервера на ESP32
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

const char* ssid = "ESP_AUTOPILOT";
const char* password = "12345678";

AsyncWebServer server(80);

// Хранение точек маршрута
struct Waypoint {
    int id;
    float x;
    float y;
};
std::vector<Waypoint> waypoints;
int waypointCounter = 0;
float mapWidth = 100.0;
float mapHeight = 100.0;

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
    </style>
    <script>
        let waypoints = [];
        let mapWidth = 100;
        let mapHeight = 100;
        let drone = { x: 50, y: 50 };

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
    <h4>Координаты дрона</h4>
    <input id="droneX" type="number" step="0.1" placeholder="X координата">
    <input id="droneY" type="number" step="0.1" placeholder="Y координата">
    <button id="updateDrone">Обновить координаты</button>
    <h4>Размер карты</h4>
        <label>Ширина: <input type="number" id="mapWidth" value="100" style="margin-right: 10px;"></label>
        <label>Высота: <input type="number" id="mapHeight" value="100" style="margin-right: 10px;"></label>
        <button id="setMapSize">Применить</button>
    <h4>Точки маршрута</h4>
    <div id="waypoints"></div>
</div>
</body>
</html>
)rawliteral";

void setup() {
    Serial.begin(115200);

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

    // Главная страница
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/html", index_html);
    });

    server.begin();
}

void loop() {
}
