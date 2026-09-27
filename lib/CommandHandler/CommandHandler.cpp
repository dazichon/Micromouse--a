#include "CommandHandler.h"
#include "VL53Sensors.h"
#include "MPU6050.h"
#include "encoder.h"

// global objects used by the current robot implementation
extern VL53Sensors tof;
extern MPU6050 gyro;

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
        if (robotNav.startExplore()) bleManager.println(">> [BLE] START EXPLORE");
        else bleManager.println(">> [BLE] Toc do = 0, tang toc do truoc khi kham pha");
    } else if (cmd == "UPDATE" || cmd == "UPDATE_MAP") {
        // chạy lại mê cung, giữ bản đồ cũ và sửa những tường khác đi
        if (robotNav.state() == NAV_EXPLORING || robotNav.state() == NAV_FAST_RUN || robotNav.state() == NAV_PID_TEST || robotNav.state() == NAV_PID_TEST_CELL)
            bleManager.println(">> [BLE] Dang chay, gui STOP truoc khi CAP NHAT");
        else if (robotNav.startUpdate()) bleManager.println(">> [BLE] START CAP NHAT ME CUNG");
        else bleManager.println(">> [BLE] Toc do = 0, tang toc do truoc khi cap nhat");
    } else if (cmd == "FAST" || cmd == "FASTRUN") {
        robotNav.startFastRun();
        bleManager.println(">> [BLE] START FAST RUN");
    } else if (cmd == "PIDTEST" || cmd == "TEST_PID") {
        // Chế độ 2: chạy PID bám tường liên tục, TỰ DO (không theo bản đồ) để test Kp/Ki/Kd live
        if (robotNav.state() == NAV_EXPLORING || robotNav.state() == NAV_FAST_RUN)
            bleManager.println(">> [BLE] Dang chay, gui STOP truoc khi test PID");
        else if (robotNav.startPidTest()) bleManager.println(">> [BLE] START TEST PID (chay tu do)");
        else bleManager.println(">> [BLE] Toc do = 0, tang toc do (DSPD) truoc khi test PID");
    } else if (cmd == "PIDCELL" || cmd == "TEST_PID_CELL") {
        // Chế độ 2 (biến thể): chạy PID bám tường TỪNG Ô (dùng CELL đã chỉnh), vẫn không theo bản đồ
        if (robotNav.state() == NAV_EXPLORING || robotNav.state() == NAV_FAST_RUN)
            bleManager.println(">> [BLE] Dang chay, gui STOP truoc khi test PID");
        else if (robotNav.startPidTestCell()) bleManager.println(">> [BLE] START TEST PID TUNG O");
        else bleManager.println(">> [BLE] Toc do = 0, tang toc do (DSPD) truoc khi test PID");
    } else if (cmd == "STOP" || cmd == "ST") {
        robotNav.stop();
        bleManager.println(">> [BLE] ROBOT STOPPED");
    } else if (cmd == "RESET" || cmd == "RESET_MAP") {
        if (robotNav.state() == NAV_EXPLORING || robotNav.state() == NAV_FAST_RUN || robotNav.state() == NAV_PID_TEST || robotNav.state() == NAV_PID_TEST_CELL) {
            bleManager.println(">> [BLE] Dang chay, gui STOP truoc khi RESET");
        } else {
            robotNav.resetAll();
            bleManager.println(">> [BLE] DA RESET TOAN BO BAN DO");
        }
    } else if (cmd == "STATUS" || cmd == "GET") {
        printStatus();
        sendParams();
        sendMazeConfig();
    } else if (cmd == "GYRO_CAL") {
        if (robotNav.state() == NAV_EXPLORING || robotNav.state() == NAV_FAST_RUN || robotNav.state() == NAV_PID_TEST || robotNav.state() == NAV_PID_TEST_CELL) {
            bleManager.println(">> [BLE] Dang chay, gui STOP truoc khi hieu chuan gyro");
        } else {
            gyro.calibrate();
            bleManager.println(">> [BLE] Da hieu chuan gyro, offset = " + String(gyro.biasDps(), 2) + " do/s");
        }
    } else if (cmd == "TURN_L" || cmd == "TURN_R" || cmd == "TURN_180") {
        // Chế độ 1: quay thử, kết quả gửi về dạng {"type":"turn",...}
        int dir = (cmd == "TURN_L") ? -1 : +1;
        float deg = (cmd == "TURN_180") ? 180.0f : 90.0f;
        if (!robotNav.testTurn(dir, deg))
            bleManager.println(">> [BLE] Dang chay me cung, gui STOP truoc khi test quay");
    } else if (cmd == "STEP_F" || cmd == "STEP_L" || cmd == "STEP_R" || cmd == "STEP_B") {
        // Chế độ 3: điều khiển tay từng ô — F thẳng, L trái+thẳng, R phải+thẳng, B quay đầu
        String err = robotNav.manualStep(cmd[5]);
        if (err.length()) bleManager.println(">> [BLE] " + err);
        else bleManager.println(">> [BLE] " + cmd + " xong, o (" + String(robotNav.cellX()) + "," +
                                String(robotNav.cellY()) + ") " + String(robotNav.headingChar()) +
                                ", encoder dem " + String(robotNav.lastDrivePulses()) + " xung");
    } else if (cmd.startsWith("SET_MAZE=")) {
        // SET_MAZE=<số ô>,<hàng đích>,<cột đích>,<cỡ vùng đích 1|2>   vd. SET_MAZE=16,8,8,2
        int n, gr, gc, gs;
        if (sscanf(cmd.c_str() + 9, "%d,%d,%d,%d", &n, &gr, &gc, &gs) != 4) {
            bleManager.println(">> [BLE] Sai cu phap. Vd: SET_MAZE=5,3,3,1");
        } else {
            String err = robotNav.setMazeConfig(n, gr, gc, gs);
            if (err.length()) bleManager.println(">> [BLE] Loi cau hinh me cung: " + err);
            else bleManager.println(">> [BLE] Da doi me cung " + String(n) + "x" + String(n));
        }
        sendMazeConfig();   // luôn gửi lại để web hiện đúng cấu hình robot đang dùng
    } else if (cmd.startsWith("SET_") && cmd.indexOf('=') > 0) {
        String key = cmd.substring(4, cmd.indexOf('='));
        float val = cmd.substring(cmd.indexOf('=') + 1).toFloat();
        float lc = robotNav.leftComp(), rc = robotNav.rightComp();
        int spd = robotNav.turnSpeed();
        if (key == "LC") lc = val;
        else if (key == "RC") rc = val;
        else if (key == "SPD") spd = (int)val;
        else if (key == "DSPD") { robotNav.setDriveSpeed((int)val); sendParams(); return; }
        else if (key == "CELL") { robotNav.setCellPulses((int)lroundf(val)); sendParams(); return; }   // độ dài 1 ô (xung encoder)
        else if (key == "YAWSCALE") {
            // hiệu chỉnh gyro báo ít/nhiều hơn góc quay thật — xem giải thích ở RobotConfig.h
            gyro.setYawScale(val);
            bleManager.println(">> [BLE] Yaw scale = " + String(gyro.yawScale(), 3));
            sendParams();
            return;
        }
        else if (key == "KP" || key == "KI" || key == "KD") {
            // 1 PID DUY NHẤT dùng cho mọi trường hợp (bám tường/gyro+encoder) — xem wallFollowCorrection()
            PIDController& w = robotNav.wallPid();
            if (key == "KP") w.setGains(val, w.getKi(), w.getKd());
            else if (key == "KI") w.setGains(w.getKp(), val, w.getKd());
            else w.setGains(w.getKp(), w.getKi(), val);
            bleManager.println(">> [BLE] PID bam tuong Kp=" + String(w.getKp(), 2) + " Ki=" + String(w.getKi(), 3) + " Kd=" + String(w.getKd(), 2));
            return;
        }
        else { bleManager.println(">> [BLE] Chua ho tro " + key); return; }
        robotNav.setTurnParams(lc, rc, spd);
        sendParams();
    }
}

void CommandHandler::sendMazeConfig() {
    String jsonMsg = "{\"type\":\"maze_cfg\",\"n\":" + String(robotNav.mazeCells()) +
                     ",\"gr\":" + String(robotNav.goalRow()) +
                     ",\"gc\":" + String(robotNav.goalCol()) +
                     ",\"gs\":" + String(robotNav.goalSize()) + "}";
    bleManager.println(jsonMsg);
}

void CommandHandler::sendParams() {
    String jsonMsg = "{\"type\":\"params\",\"lc\":" + String(robotNav.leftComp(), 1) +
                     ",\"rc\":" + String(robotNav.rightComp(), 1) +
                     ",\"spd\":" + String(robotNav.turnSpeed()) +
                     ",\"dspd\":" + String(robotNav.driveSpeed()) +
                     ",\"cell\":" + String(robotNav.cellPulses()) +
                     ",\"yawscale\":" + String(gyro.yawScale(), 3) + "}";
    bleManager.println(jsonMsg);
}

void CommandHandler::sendTelemetry() {
    // vừa kết nối => gửi tham số quay hiện tại để dashboard đồng bộ thanh trượt
    bool connected = bleManager.isConnected();
    if (connected && !_wasConnected) { sendParams(); sendMazeConfig(); }
    _wasConnected = connected;

    String event;
    if (connected && robotNav.takeEvent(event)) bleManager.println(">> [ROBOT] " + event);

    float target, turned;
    char src;
    if (connected && robotNav.takeTurnResult(target, turned, src)) {
        String jsonMsg = "{\"type\":\"turn\",\"target\":" + String(target, 0) +
                         ",\"turned\":" + String(turned, 1) +
                         ",\"src\":\"" + String(src) + "\"}";
        bleManager.println(jsonMsg);
    }

    if (bleManager.isConnected() && millis() - _lastTelemetryTime >= 200) {
        _lastTelemetryTime = millis();

        WallDist d = tof.read();
        float yaw = robotNav.displayYawDeg();

        String jsonMsg = "{\"type\":\"telemetry\",\"yaw\":" + String(yaw, 1) +
                         ",\"gr\":" + String(gyro.rateDps(), 2) +
                         ",\"gb\":" + String(gyro.biasDps(), 2) +
                         ",\"l\":" + String(d.left_mm) +
                         ",\"f\":" + String(d.front_mm) +
                         ",\"r\":" + String(d.right_mm) +
                         ",\"ok\":" + String(d.left_ok || d.front_ok || d.right_ok ? 1 : 0) +
                         ",\"el\":" + String(encoder.left()) +
                         ",\"er\":" + String(encoder.right()) +
                         ",\"dp\":" + String(robotNav.lastDrivePulses()) +
                         ",\"st\":" + String((int)robotNav.state()) +
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
    float yaw = robotNav.displayYawDeg();

    String statusMsg = "--- STATS ---\n";
    statusMsg += "Yaw: " + String(yaw, 2) + " deg\n";
    statusMsg += "TOF (L/F/R): " + String(d.left_mm) + " / " + String(d.front_mm) + " / " + String(d.right_mm) + " mm\n";
    statusMsg += "STATE: " + String(robotNav.state()) + "\n";
    statusMsg += "POS: (" + String(robotNav.cellX()) + ", " + String(robotNav.cellY()) + ")";

    Serial.println(statusMsg);
    bleManager.println(statusMsg);
}
