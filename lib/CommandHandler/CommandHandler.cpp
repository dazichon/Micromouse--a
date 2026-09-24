#include "CommandHandler.h"
#include "VL53Sensors.h"
#include "MPU6050.h"

// global objects used by the current robot implementation
extern VL53Sensors tof;
extern MPU6050Gyro gyro;

CommandHandler commandHandler;

CommandHandler::CommandHandler() {
    _lastTelemetryTime = 0;
}

void CommandHandler::update() {
    processBLECommands();
    sendTelemetry();
}

void CommandHandler::processBLECommands() {
    if (!bleManager.hasCommand()) return;

    String cmd = bleManager.readCommand();
    cmd.toUpperCase();

    if (cmd == "START" || cmd == "EXPLORE") {
        robotNav.startExplore();
        bleManager.println(">> [BLE] START EXPLORE");
    } else if (cmd == "FAST" || cmd == "FASTRUN") {
        robotNav.startFastRun();
        bleManager.println(">> [BLE] START FAST RUN");
    } else if (cmd == "STOP" || cmd == "ST") {
        robotNav.stop();
        bleManager.println(">> [BLE] ROBOT STOPPED");
    } else if (cmd == "STATUS" || cmd == "GET") {
        printStatus();
    }
}

void CommandHandler::sendTelemetry() {
    if (bleManager.isConnected() && millis() - _lastTelemetryTime >= 200) {
        _lastTelemetryTime = millis();

        WallDist d = tof.read();
        float yaw = gyro.yawDeg();

        String jsonMsg = "{\"type\":\"telemetry\",\"yaw\":" + String(yaw, 1) +
                         ",\"l\":" + String(d.left_mm) +
                         ",\"f\":" + String(d.front_mm) +
                         ",\"r\":" + String(d.right_mm) +
                         ",\"ok\":" + String(d.left_ok || d.front_ok || d.right_ok ? 1 : 0) +
                         "}";
        bleManager.println(jsonMsg);
    }

    if (bleManager.isConnected() && robotNav.hasMapChanged()) {
        robotNav.clearMapChanged();
        String mapData = robotNav.getMazeString();
        String jsonMsg = "{\"type\":\"map\",\"cx\":" + String(robotNav.cellX()) + 
                         ",\"cy\":" + String(robotNav.cellY()) + 
                         ",\"dir\":\"" + String(robotNav.headingChar()) + "\"" +
                         ",\"data\":\"" + mapData + "\"}";
        bleManager.println(jsonMsg);
    }
}

void CommandHandler::printStatus() {
    WallDist d = tof.read();
    gyro.update();
    float yaw = gyro.yawDeg();

    String statusMsg = "--- STATS ---\n";
    statusMsg += "Yaw: " + String(yaw, 2) + " deg\n";
    statusMsg += "TOF (L/F/R): " + String(d.left_mm) + " / " + String(d.front_mm) + " / " + String(d.right_mm) + " mm\n";
    statusMsg += "STATE: " + String(robotNav.state()) + "\n";
    statusMsg += "POS: (" + String(robotNav.cellX()) + ", " + String(robotNav.cellY()) + ")";

    Serial.println(statusMsg);
    bleManager.println(statusMsg);
}
