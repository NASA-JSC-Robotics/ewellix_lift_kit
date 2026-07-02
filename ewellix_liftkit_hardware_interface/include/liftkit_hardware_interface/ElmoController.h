#ifndef ELMO_CONTROLLER_H
#define ELMO_CONTROLLER_H

#include <string>
#include <stdexcept>
#include <thread>
#include <mutex>
#include <chrono>
#include <cstdint>

using namespace std; 

class ElmoController {
private:
    string port_name;
    uint32_t baud_rate;
    int fd;                     // File descriptor (replaces HANDLE)
    mutex port_mutex;

    void setSerialAttributes();
    string sendCommandAndRead(const string& cmd, int timeout_ms = 500);

public:
    ElmoController(const string& port, uint32_t baud = 115200);
    ~ElmoController();
    void waitForMotionComplete(int timeout_ms = 5000);
    void connect();
    void disconnect();
    bool isConnected() const;

    void motorOn();
    void motorOff();

    void setVelocityMode();
    void setPositionMode();
    void setCurrentMode();

    void setVelocity(int32_t velocity);
    void setPosition(int32_t position);
    void setCurrent(float current);

    void setPositionRelative(int32_t delta);
    void beginMotion();
    void stopMotion();

    void velocityForTime(int32_t velocity, int duration_ms, int poll_ms = 100);
    void currentForTime(float current, int duration_ms, int poll_ms = 100);

    int32_t getPosition();
    int32_t getVelocity();
    float   getCurrent();
    int32_t getStatus();

    void wait(int milliseconds);
    void sendRawCommand(const string& cmd);
    string readRawResponse();

    string getSerialNumber();

    float getElmoTemperature();
    
};

#endif // ELMO_CONTROLLER_H