// Tramitador PAMI - Sistema de control de servos con WiFi
// ESP32 Servo Control with Web Interface

#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>

// Configuración WiFi - Cambiar por tus datos
const char* ssid = "UCC-Libre";     // Cambiar por el nombre de tu red
const char* password = "";  // Cambiar por tu contraseña

// Configuración del servo
Servo myServo;
int servoPin = 18;  // Pin de control del servo
int currentPosition = 90;  // Posición actual del servo
int minUs = 500;
int maxUs = 2400;

// Servidor web
WebServer server(80);

// Variables de estado
bool autoMode = false;
unsigned long lastMoveTime = 0;
int autoDirection = 1;

// Declaraciones de funciones
void setupWebServer();
String getWebInterface();

void setup() {
  Serial.begin(9600);
  delay(1000);

  // Inicializar SPIFFS
  if(!SPIFFS.begin(true)){
    Serial.println("Error montando SPIFFS");
  }

  // Configurar servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  myServo.setPeriodHertz(50);
  int result = myServo.attach(servoPin, minUs, maxUs);
  
  if (result) {
    Serial.print("Servo inicializado en pin ");
    Serial.println(servoPin);
    myServo.write(currentPosition);
  } else {
    Serial.println("ERROR: Fallo al inicializar servo");
  }

  // Conectar WiFi
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("WiFi conectado! IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("No se pudo conectar a WiFi, usando modo AP");
    WiFi.softAP("TramitadorPAMI", "12345678");
    Serial.print("IP del Access Point: ");
    Serial.println(WiFi.softAPIP());
    Serial.println(WiFi.softAPIPv6());
  }

  // Configurar rutas del servidor web
  setupWebServer();
  
  server.begin();
  Serial.println("Servidor web iniciado");
  Serial.println("Accede a la interfaz web desde tu navegador");
}

void loop() {
  server.handleClient();
  
  // Modo automático
  if (autoMode && millis() - lastMoveTime > 50) {
    currentPosition += autoDirection;
    if (currentPosition >= 180) {
      currentPosition = 180;
      autoDirection = -1;
    } else if (currentPosition <= 0) {
      currentPosition = 0;
      autoDirection = 1;
    }
    
    if (myServo.attached()) {
      myServo.write(currentPosition);
    }
    lastMoveTime = millis();
  }
  
  delay(10);
}

void setupWebServer() {
  // Página principal
  server.on("/", HTTP_GET, []() {
    String html = getWebInterface();
    server.send(200, "text/html", html);
  });
  
  // API para mover el servo
  server.on("/move", HTTP_GET, []() {
    if (server.hasArg("pos")) {
      int pos = server.arg("pos").toInt();
      if (pos >= 0 && pos <= 180) {
        currentPosition = pos;
        if (myServo.attached()) {
          myServo.write(currentPosition);
        }
        Serial.println("Servo movido a: " + String(currentPosition) + "°");
        server.send(200, "application/json", "{\"status\":\"ok\",\"position\":" + String(currentPosition) + "}");
      } else {
        server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Posición inválida\"}");
      }
    } else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Falta parámetro pos\"}");
    }
  });
  
  // API para obtener estado
  server.on("/status", HTTP_GET, []() {
    String status = "{\"position\":" + String(currentPosition) + 
                   ",\"autoMode\":" + (autoMode ? "true" : "false") + 
                   ",\"connected\":" + (myServo.attached() ? "true" : "false") + "}";
    server.send(200, "application/json", status);
  });
  
  // API para modo automático
  server.on("/auto", HTTP_GET, []() {
    if (server.hasArg("enable")) {
      autoMode = server.arg("enable") == "true";
      String response = "{\"status\":\"ok\",\"autoMode\":" + String(autoMode ? "true" : "false") + "}";
      server.send(200, "application/json", response);
      Serial.println("Modo automático: " + String(autoMode ? "ON" : "OFF"));
    } else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Falta parámetro enable\"}");
    }
  });
}

String getWebInterface() {
  return R"html(
<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Tramitador PAMI - Control de Servo</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            max-width: 800px;
            margin: 0 auto;
            padding: 20px;
            background-color: #f0f8ff;
        }
        .header {
            text-align: center;
            color: #2c5aa0;
            border-bottom: 3px solid #2c5aa0;
            padding-bottom: 10px;
            margin-bottom: 30px;
        }
        .control-panel {
            background: white;
            border-radius: 10px;
            padding: 20px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
            margin-bottom: 20px;
        }
        .slider-container {
            margin: 20px 0;
        }
        .slider {
            width: 100%;
            height: 25px;
            border-radius: 5px;
            background: #d3d3d3;
            outline: none;
            opacity: 0.7;
            transition: opacity 0.2s;
        }
        .slider:hover {
            opacity: 1;
        }
        .position-display {
            font-size: 2em;
            font-weight: bold;
            text-align: center;
            color: #2c5aa0;
            margin: 20px 0;
        }
        .button {
            background-color: #2c5aa0;
            border: none;
            color: white;
            padding: 15px 32px;
            text-align: center;
            text-decoration: none;
            display: inline-block;
            font-size: 16px;
            margin: 4px 2px;
            cursor: pointer;
            border-radius: 5px;
            transition: background-color 0.3s;
        }
        .button:hover {
            background-color: #1e3d72;
        }
        .button.auto-on {
            background-color: #28a745;
        }
        .button.auto-on:hover {
            background-color: #218838;
        }
        .status {
            background-color: #e9ecef;
            padding: 10px;
            border-radius: 5px;
            margin: 10px 0;
        }
        .preset-buttons {
            display: flex;
            justify-content: space-around;
            flex-wrap: wrap;
            gap: 10px;
            margin: 20px 0;
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>🏥 Tramitador PAMI</h1>
        <p>Sistema de Control de Servomotor</p>
    </div>

    <div class="control-panel">
        <h2>Control Manual</h2>
        <div class="position-display" id="positionDisplay">90°</div>
        
        <div class="slider-container">
            <input type="range" min="0" max="180" value="90" class="slider" id="positionSlider">
        </div>
        
        <div class="preset-buttons">
            <button class="button" onclick="moveToPosition(0)">0°</button>
            <button class="button" onclick="moveToPosition(45)">45°</button>
            <button class="button" onclick="moveToPosition(90)">90°</button>
            <button class="button" onclick="moveToPosition(135)">135°</button>
            <button class="button" onclick="moveToPosition(180)">180°</button>
        </div>
    </div>

    <div class="control-panel">
        <h2>Modo Automático</h2>
        <button class="button" id="autoButton" onclick="toggleAuto()">Activar Auto</button>
        <div class="status" id="statusDisplay">
            Estado: Manual
        </div>
    </div>

    <script>
        const slider = document.getElementById('positionSlider');
        const positionDisplay = document.getElementById('positionDisplay');
        const autoButton = document.getElementById('autoButton');
        const statusDisplay = document.getElementById('statusDisplay');
        
        let isAutoMode = false;
        let currentPosition = 90;
        
        // Actualizar posición cuando se mueve el slider
        slider.oninput = function() {
            const pos = this.value;
            moveToPosition(pos);
        }
        
        function moveToPosition(pos) {
            fetch('/move?pos=' + pos)
                .then(response => response.json())
                .then(data => {
                    if (data.status === 'ok') {
                        currentPosition = data.position;
                        updateDisplay();
                    }
                })
                .catch(error => console.error('Error:', error));
        }
        
        function toggleAuto() {
            isAutoMode = !isAutoMode;
            fetch('/auto?enable=' + isAutoMode)
                .then(response => response.json())
                .then(data => {
                    if (data.status === 'ok') {
                        updateAutoButton();
                    }
                })
                .catch(error => console.error('Error:', error));
        }
        
        function updateDisplay() {
            positionDisplay.textContent = currentPosition + '°';
            slider.value = currentPosition;
        }
        
        function updateAutoButton() {
            if (isAutoMode) {
                autoButton.textContent = 'Desactivar Auto';
                autoButton.className = 'button auto-on';
                statusDisplay.textContent = 'Estado: Automático - Movimiento continuo';
            } else {
                autoButton.textContent = 'Activar Auto';
                autoButton.className = 'button';
                statusDisplay.textContent = 'Estado: Manual - Control directo';
            }
        }
        
        // Actualizar estado cada segundo
        setInterval(function() {
            fetch('/status')
                .then(response => response.json())
                .then(data => {
                    currentPosition = data.position;
                    isAutoMode = data.autoMode;
                    updateDisplay();
                    updateAutoButton();
                })
                .catch(error => console.error('Error:', error));
        }, 1000);
        
        // Inicializar interfaz
        updateDisplay();
        updateAutoButton();
    </script>
</body>
</html>
)html";
}
