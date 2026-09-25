#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <Arduino.h>
#include "BLEManager.h"
#include "RobotNav.h"

class CommandHandler {
public:
    CommandHandler();

    void update();
    void processBLECommands();
    void sendTelemetry();
    void printStatus();
    void sendParams();
    void sendMazeConfig();

private:
    unsigned long _lastTelemetryTime;
    bool _wasConnected = false;
};

extern CommandHandler commandHandler;

#endif
