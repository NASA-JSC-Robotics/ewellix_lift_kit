#ifndef ELMO_CONTROLLER_H
#define ELMO_CONTROLLER_H

#include <string>
#include <stdexcept>
#include <thread>
#include <mutex>
#include <chrono>
#include <cstdint>

using namespace std;


// Homing configuration
constexpr float   HOMING_CURRENT_A       = 0.5f;   // amps, well under CL[1]
constexpr int32_t STALL_VELOCITY_THRESH  = 5;       // counts/sec
constexpr int     STALL_TIME_MS          = 400;     // dwell before declaring hard stop
constexpr int     HOMING_TIMEOUT_MS      = 15000;   // abort if never stalls
constexpr int     POLL_MS                = 20;
constexpr int32_t BACKOFF_COUNTS         = 500;     // pull off the hard stop after homing

// Direction each motor must drive to reach its home (hard stop).
// Adjust signs to match your actual mechanical/encoder convention.
constexpr int BOTTOM_HOME_DIRECTION = 1;  // e.g. drive down/retract
constexpr int TOP_HOME_DIRECTION    = 1;  // e.g. drive up/extend

class ElmoController {
private:
    string port_name;
    uint32_t baud_rate;
    int fd;                     // File descriptor (replaces HANDLE)
    mutex port_mutex;

    void setSerialAttributes();
    

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
    void disableEcho();
    void enableEcho();
    // Homing
    bool homeToHardStop(float current, int direction,
                         int32_t stall_velocity_threshold = STALL_VELOCITY_THRESH,
                         int stall_time_ms = STALL_TIME_MS,
                         int timeout_ms = HOMING_TIMEOUT_MS,
                         int poll_ms = POLL_MS,
                         int32_t backoff_counts = BACKOFF_COUNTS);
    void zeroPosition();

    // Static flag to enable delays for calibration/homing operations
    static bool calibration_mode;
    
    // Method to set calibration mode
    static void setCalibrationMode(bool enabled) {
        calibration_mode = enabled;
    }

    string sendCommandAndRead(const string& cmd, int timeout_ms = 4);
};

#endif // ELMO_CONTROLLER_H