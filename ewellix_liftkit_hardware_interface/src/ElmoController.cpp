/* Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * This software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include "liftkit_hardware_interface/ElmoController.h"

#include <iostream>
#include <cstring>

#include <fcntl.h>      
#include <unistd.h>     
#include <termios.h>    
#include <errno.h>
#include <sys/select.h> 

using namespace std; 

bool ElmoController::calibration_mode = false;
uint32_t baud = B115200; // Highest supported baud rate via USB for Elmo controllers.

/**
 * Constructor for ElmoController object, sets up serial connection on startup.
 */
ElmoController::ElmoController(const string& port, uint32_t baud)
    : port_name(port), baud_rate(baud), fd(-1) {}

/**
 * Destructor for ElmoController object, disconnects and closes serial port to Elmo controllers.
 */
ElmoController::~ElmoController() {
    if (isConnected()) {
        try { 
            disconnect(); 
        } 
        
        catch (...) {}
    }
}


/**
 * Connects to Elmo controller with serial flags:
 * 
 * O_RDWR = Open for read and write.
 * O_NOCTTY = Opens serial device but as non-controlling terminal.
 * O_NDELAY = Non-blocking wait for initial serial connection.
 * F_GETFL = Current flags for file descriptor.
 * ~O_NONBLOCK = Disables non-blocking mode and enables blocking.
 */
void ElmoController::connect() {
    fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        throw runtime_error("Failed to open serial port '" +
                                 port_name + "': " + strerror(errno));
    }

    // Switch fd back to blocking I/O
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    // Specific attributes needed for Elmo motor control commands.
    setSerialAttributes();
}

/**
 * Specific attributes needed for Elmo motor controller commands.
 * Goal is to not modify the Elmo command packet due to serial attributes.
 */
void ElmoController::setSerialAttributes() {
    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        fd = -1;
        throw runtime_error(string("tcgetattr failed: ") +
                                 strerror(errno));
    }

    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    // Configure the serial port to match the Elmo controller's communication settings: 
    // 8N1, no hardware flow control, receiver enabled.
    tty.c_cflag &= ~PARENB; 
    tty.c_cflag &= ~CSTOPB; 
    tty.c_cflag &= ~CSIZE;            
    tty.c_cflag |=  CS8; 
    tty.c_cflag &= ~CRTSCTS; 
    tty.c_cflag |=  CREAD | CLOCAL; 

    // Local flags
    // Raw mode: disable terminal processing so bytes are transmitted and received exactly as sent. 
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // Disable software flow control and input processing so received bytes
    // are passed to the application unchanged.
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); 
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK |
                     ISTRIP | INLCR | IGNCR | ICRNL);

    // Disable output processing so command bytes are transmitted unchanged.
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VMIN]  = 0;  // Return as soon as any data arrives
    tty.c_cc[VTIME] = 0;  
    
    // Apply terminal settings, close port if failure.
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        fd = -1;
        throw runtime_error(string("tcsetattr failed: ") +
                                 strerror(errno));
    }

    tcflush(fd, TCIOFLUSH); // Flush any stale data
}

/**
 * Close serial connection.
 */
void ElmoController::disconnect() {
    if (fd >= 0) {
        close(fd);
        fd = -1;
    }
}

/**
 * Checks if there is a serial connection.
 */
bool ElmoController::isConnected() const {
    return fd >= 0;
}

/**
 * Sends and recieves Elmo controller motor commands via serial. Handles data parsing according to Elmo manual.
 *
 * Example Flow:
 * 1. HW Interface sends "PX\r" to Elmo.
 * 2. Elmo echoes back "PX;PX\r" + "data\r".
 * 3. Current position recieved once parsed as just "data".
 */
string ElmoController::sendCommandAndRead(const string& cmd, int timeout_ms) {
    lock_guard<mutex> lock(port_mutex);

    if (!isConnected()) throw runtime_error("Serial port not open!");

    string full_cmd = cmd + "\r";
    ssize_t written = write(fd, full_cmd.c_str(), full_cmd.size());
    if (written < 0) {
        throw runtime_error(string("write() failed: ") + strerror(errno));
    }
    
    string response;
    auto start = chrono::steady_clock::now();

    while (chrono::duration_cast<chrono::milliseconds>(
               chrono::steady_clock::now() - start).count() < timeout_ms) {

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        struct timeval tv;
        int remaining_ms = timeout_ms - 
            (int)chrono::duration_cast<chrono::milliseconds>(
                chrono::steady_clock::now() - start).count();
        if (remaining_ms <= 0) break;
        tv.tv_sec  = remaining_ms / 1000;
        tv.tv_usec = (remaining_ms % 1000) * 1000;

        int ret = select(fd + 1, &rfds, nullptr, nullptr, &tv);
        if (ret <= 0) break;

        string line;
        char byte;
        while (chrono::duration_cast<chrono::milliseconds>(
                   chrono::steady_clock::now() - start).count() < timeout_ms) {

            FD_ZERO(&rfds);
            FD_SET(fd, &rfds);
            int rem2 = timeout_ms - 
                (int)chrono::duration_cast<chrono::milliseconds>(
                    chrono::steady_clock::now() - start).count();
            if (rem2 <= 0) break;
            tv.tv_sec  = rem2 / 1000;
            tv.tv_usec = (rem2 % 1000) * 1000;

            if (select(fd + 1, &rfds, nullptr, nullptr, &tv) <= 0) break;

            ssize_t n = read(fd, &byte, 1);
            if (n <= 0) break;

            if (byte == '\r' || byte == '\n') {
                if (!line.empty()) break;
            } else {
                line += byte;
            }
        }

        if (line.empty()) continue;

        // Trim whitespace
        auto s = line.find_first_not_of(" \t\r\n");
        if (s == string::npos) continue;
        line = line.substr(s);
        auto e = line.find_last_not_of(" \t\r\n");
        if (e != string::npos) line.erase(e + 1);

        // NEW: Check for error response (starts with digit or '?')
        // With EO=0, we get response directly without echo
        if (!line.empty() && 
            (isdigit((unsigned char)line[0]) ||
             line[0] == '-' || line[0] == '.' || line[0] == '?')) {
            response = line;
            break;
        }
        
        // If EO=1 (echo enabled), still strip ";CMD" suffix
        auto semicolon = line.find(';');
        if (semicolon != string::npos) {
            line = line.substr(0, semicolon);
            if (!line.empty() &&
                (isdigit((unsigned char)line[0]) ||
                 line[0] == '-' || line[0] == '.')) {
                response = line;
                break;
            }
        }
    }

    return response;
}

/**
 * Enables the motor.
 */
void ElmoController::motorOn() {
    try {
        sendCommandAndRead("MO=1");
    } catch (const exception& e) {
        cerr << "Error in motorOn: " << e.what() << endl;
        throw;
    }
}

/**
 * Disables the motor.
 */
void ElmoController::motorOff() {
    try {
        sendCommandAndRead("MO=0");
    } catch (const exception& e) {
        cerr << "Error in motorOff: " << e.what() << endl;
        throw;
    }
}

/**
 * Non-blocking polls Elmo every 100 ms to detect if motion has stopped. 
 * eg. if motor has reached position limit.
 */
void ElmoController::waitForMotionComplete(int timeout_ms) {
    auto start = chrono::steady_clock::now();

    while (chrono::duration_cast<chrono::milliseconds>(
               chrono::steady_clock::now() - start).count() < timeout_ms) {

        string response = sendCommandAndRead("MS");
        if (!response.empty()) {
            try {
                int status = stoi(response);
                if (status == 0 || status == 3) {
                    return;
                }
            } catch (...) {}
        }
        this_thread::sleep_for(chrono::milliseconds(100));
    }
}

/**
 * Sets Elmo in velocity mode.
 */
void ElmoController::setVelocityMode() {
    try {
        sendCommandAndRead("UM=2");
    } catch (const exception& e) {
        cerr << "Error in setVelocityMode: " << e.what() << endl;
        throw;
    }
}

/**
 * Sets Elmo in position mode.
 */
void ElmoController::setPositionMode() {
    try {
        sendCommandAndRead("UM=5");
    } catch (const exception& e) {
        cerr << "Error in setPositionMode: " << e.what() << endl;
        throw;
    }
}

/**
 * Sets Elmo in current mode.
 */
void ElmoController::setCurrentMode() {
    try {
        sendCommandAndRead("UM=1");
    } catch (const exception& e) {
        cerr << "Error in setCurrentMode: " << e.what() << endl;
        throw;
    }
}

/**
 * Spins motor at specified velocity.
 */
void ElmoController::setVelocity(int32_t velocity) {
    try {
        string cmd = "JV=" + to_string(velocity);
        sendCommandAndRead(cmd);
        beginMotion();
    } catch (const exception& e) {
        cerr << "Error in setVelocity: " << e.what() << endl;
        throw;
    }
}

/**
 * Spins motor at specified current forever.
 */
void ElmoController::setCurrent(float current) {
    try {
        string cmd = "TC=" + to_string(current);
        sendCommandAndRead(cmd);
        beginMotion();
    } catch (const exception& e) {
        cerr << "Error in setCurrent: " << e.what() << endl;
        throw;
    }
}

/**
 * Spins motor to specified encoder position.
 */
void ElmoController::setPosition(int32_t position) {
    try {
        string cmd = "PA=" + to_string(position);
        sendCommandAndRead(cmd);
    } catch (const exception& e) {
        cerr << "Error in setPosition: " << e.what() << endl;
        throw;
    }
}

/**
 * Moves motor certain distance from current position.
 */
void ElmoController::setPositionRelative(int32_t delta) {
    try {
        string cmd = "PR=" + to_string(delta);
        sendCommandAndRead(cmd);
    } catch (const exception& e) {
        cerr << "Error in setPositionRelative: " << e.what() << endl;
        throw;
    }
}

/**
  * Begins programmed motions that were configured.
  */
void ElmoController::beginMotion() {
    try {
        sendCommandAndRead("BG");
    } catch (const exception& e) {
        cerr << "Error in beginMotion: " << e.what() << endl;
        throw;
    }
}

/**
 * Stops current motion on motor.
 */
void ElmoController::stopMotion() {
    try {
        sendCommandAndRead("ST");
    } catch (const exception& e) {
        cerr << "Error in stopMotion: " << e.what() << endl;
        throw;
    }
}

/**
 * Spins motor at specified velocity for a specific amount of time.
 */
void ElmoController::velocityForTime(int32_t velocity, int duration_ms, int poll_ms) {
    try {
        setVelocity(velocity);

        auto start = chrono::steady_clock::now();
        while (chrono::duration_cast<chrono::milliseconds>(
                   chrono::steady_clock::now() - start).count() < duration_ms) {
            wait(poll_ms);
        }

        stopMotion();

    } catch (const exception& e) {
        cerr << "Error in velocityForTime: " << e.what() << endl;
        stopMotion();
        throw;
    }
}

/**
 * Spins motor at specified current for a specific amount of time.
 */
void ElmoController::currentForTime(float current, int duration_ms, int poll_ms) {
    try {
        setCurrent(current);

        auto start = chrono::steady_clock::now();
        while (chrono::duration_cast<chrono::milliseconds>(
                   chrono::steady_clock::now() - start).count() < duration_ms) {
            wait(poll_ms);
        }

        stopMotion();

    } catch (const exception& e) {
        cerr << "Error in currentForTime: " << e.what() << endl;
        stopMotion();
        throw;
    }
}

/**
 * Gets current position of the motor via encoder readings.
 */
int32_t ElmoController::getPosition() {
    try {
        string response = sendCommandAndRead("PX");
        return stoi(response);
    } catch (const exception& e) {
        cerr << "Error in getPosition: " << e.what() << endl;
        return 0;
    }
}

/**
 * Gets current velocity of the motor.
 */
int32_t ElmoController::getVelocity() {
    try {
        string response = sendCommandAndRead("VX");
        return stoi(response);
    } catch (const exception& e) {
        cerr << "Error in getVelocity: " << e.what() << endl;
        return 0;
    }
}

/**
 * Gets current draw of the motor.
 */
float ElmoController::getCurrent() {
    try {
        string response = sendCommandAndRead("IQ");
        return stof(response);
    } catch (const exception& e) {
        cerr << "Error in getCurrent: " << e.what() << endl;
        return 0.0f;
    }
}

/**
 * Gets contents of Elmo status register for enabled, fault, moving, etc.
 */
int32_t ElmoController::getStatus() {
    try {
        string response = sendCommandAndRead("SR");
        return stoi(response);
    } catch (const exception& e) {
        cerr << "Error in getStatus: " << e.what() << endl;
        return 0;
    }
}

/**
 * Gets current Elmo controller temperature.
 */
float ElmoController::getElmoTemperature() {
    try {
        string response = sendCommandAndRead("TI[1]");
        return stof(response);
    } catch (const exception& e) {
        cerr << "Error in getElmoTemperature: " << e.what() << endl;
        return -999.0f;
    }
}

/**
 * Delay per Elmo thread, useful for delaying a specific motor.
 */
void ElmoController::wait(int milliseconds) {
    this_thread::sleep_for(chrono::milliseconds(milliseconds));
}

/**
 * Useful for debugging serial transmission, stripped down sendCommandAndRead() for sending raw bytes.
 */
void ElmoController::sendRawCommand(const string& cmd) {
    lock_guard<mutex> lock(port_mutex);
    if (!isConnected()) throw runtime_error("Serial port not open!");
    string full_cmd = cmd + "\r";
    ssize_t written = write(fd, full_cmd.c_str(), full_cmd.size());
    if (written < 0) {
        throw runtime_error(string("write() failed: ") +
                                 strerror(errno));
    }
}

/**
 * Useful for debugging serial transmission, stripped down sendCommandAndRead() for 
 * reading raw bytes from Elmo without parsing.
 */
string ElmoController::readRawResponse() {
    lock_guard<mutex> lock(port_mutex);
    if (!isConnected()) throw runtime_error("Serial port not open!");
    char buffer[256];
    ssize_t n = read(fd, buffer, sizeof(buffer) - 1);
    if (n > 0) return string(buffer, n);
    return {};
}

/**
 * Gets serial number of a specific Elmo controller.
 */
string ElmoController::getSerialNumber() {
    try {
        string response = sendCommandAndRead("SN[4]");
        return response;
    } catch (const exception& e) {
        cerr << "Error in getSerialNumber: " << e.what() << endl;
        return "";
    }
}

/**
 * Zeros out encoder position.
 */
void ElmoController::zeroPosition() {
    try {
        sendCommandAndRead("PX=0");
    } catch (const exception& e) {
        cerr << "Error in zeroPosition: " << e.what() << endl;
        throw;
    }
}

/**
 * Finds mechanical hard stop of actuator that does not have built in limit switch.
 */
bool ElmoController::homeToHardStop(float current, int direction,
                                     int32_t stall_velocity_threshold,
                                     int stall_time_ms,
                                     int timeout_ms,
                                     int poll_ms,
                                     int32_t backoff_counts) {
    try {
        setCurrentMode();
        motorOn();

        float signed_current = (direction >= 0) ? current : -current;

        setCurrent(signed_current);

        auto start = chrono::steady_clock::now();
        auto stall_start = start;
        bool stalling = false;

        while (chrono::duration_cast<chrono::milliseconds>(
                   chrono::steady_clock::now() - start).count() < timeout_ms) {

            int32_t vel = getVelocity();

            if (abs(vel) < stall_velocity_threshold) {
                if (!stalling) {
                    stalling = true;
                    stall_start = chrono::steady_clock::now();
                }
                int stalled_for = (int)chrono::duration_cast<chrono::milliseconds>(
                    chrono::steady_clock::now() - stall_start).count();

                if (stalled_for >= stall_time_ms) {
                    stopMotion();
                    zeroPosition();

                    if (backoff_counts != 0) {
                        setPositionMode();
                        motorOn();
                        setPositionRelative(-direction * abs(backoff_counts));
                        beginMotion();
                        waitForMotionComplete();
                    }

                    return true;
                }
            } else {
                stalling = false;
            }

            wait(poll_ms);
        }

        stopMotion();
        return false;

    } catch (const exception& e) {
        cerr << "Error in homeToHardStop: " << e.what() << endl;
        stopMotion();
        throw;
    }
}

void ElmoController::disableEcho() {
    try {
        sendCommandAndRead("EO=0");
    } catch (const exception& e) {
        cerr << "Error disabling echo: " << e.what() << endl;
        throw;
    }
}

void ElmoController::enableEcho() {
    try {
        sendCommandAndRead("EO=1");
    } catch (const exception& e) {
        cerr << "Error enabling echo: " << e.what() << endl;
        throw;
    }
}