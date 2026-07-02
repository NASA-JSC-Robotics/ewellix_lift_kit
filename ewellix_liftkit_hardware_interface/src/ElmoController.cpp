#include "liftkit_hardware_interface/ElmoController.h"

#include <iostream>
#include <cstring>

// POSIX / Linux serial headers
#include <fcntl.h>      // open(), O_RDWR, O_NOCTTY, O_NDELAY
#include <unistd.h>     // close(), read(), write()
#include <termios.h>    // struct termios, tcgetattr(), tcsetattr(), cfsetspeed()
#include <errno.h>
#include <sys/select.h> // select() for timeout reads

using namespace std; 

// ---------------------------------------------------------------------------
// Helper: map integer baud rate to termios speed_t constant
// ---------------------------------------------------------------------------
static speed_t baudRateToSpeed(uint32_t baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:
            throw runtime_error("Unsupported baud rate: " +
                                     to_string(baud));
    }
}

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------
ElmoController::ElmoController(const string& port, uint32_t baud)
    : port_name(port), baud_rate(baud), fd(-1) {}

ElmoController::~ElmoController() {
    if (isConnected()) {
        try { disconnect(); } catch (...) {}
    }
}

// ---------------------------------------------------------------------------
// connect()
// ---------------------------------------------------------------------------
void ElmoController::connect() {
    // O_NOCTTY  : don't make this the controlling terminal
    // O_NDELAY  : non-blocking open (we set blocking later via termios)
    fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        throw runtime_error("Failed to open serial port '" +
                                 port_name + "': " + strerror(errno));
    }

    // Switch fd back to blocking I/O
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    setSerialAttributes();

    cout << "✓ Connected to Elmo drive on " << port_name
              << " at " << baud_rate << " baud" << endl;
}

// ---------------------------------------------------------------------------
// setSerialAttributes()  – configure termios (8N1, raw mode)
// ---------------------------------------------------------------------------
void ElmoController::setSerialAttributes() {
    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        fd = -1;
        throw runtime_error(string("tcgetattr failed: ") +
                                 strerror(errno));
    }

    speed_t spd = baudRateToSpeed(baud_rate);
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);

    // --- Control flags ---
    tty.c_cflag &= ~PARENB;          // No parity
    tty.c_cflag &= ~CSTOPB;          // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |=  CS8;             // 8 data bits
    tty.c_cflag &= ~CRTSCTS;         // No hardware flow control
    tty.c_cflag |=  CREAD | CLOCAL;  // Enable receiver, ignore modem lines

    // --- Local flags: raw mode ---
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // --- Input flags ---
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK |
                     ISTRIP | INLCR | IGNCR | ICRNL);

    // --- Output flags: raw output ---
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    // --- VMIN / VTIME: blocking read with 100 ms inter-byte timeout ---
    tty.c_cc[VMIN]  = 0;  // Return as soon as any data arrives…
    tty.c_cc[VTIME] = 1;  // …or after 0.1 s (units of 0.1 s)

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        fd = -1;
        throw runtime_error(string("tcsetattr failed: ") +
                                 strerror(errno));
    }

    tcflush(fd, TCIOFLUSH); // Flush any stale data
}

// ---------------------------------------------------------------------------
// disconnect()
// ---------------------------------------------------------------------------
void ElmoController::disconnect() {
    if (fd >= 0) {
        close(fd);
        fd = -1;
        cout << "Disconnected from Elmo drive" << endl;
    }
}

// ---------------------------------------------------------------------------
// isConnected()
// ---------------------------------------------------------------------------
bool ElmoController::isConnected() const {
    return fd >= 0;
}

// ---------------------------------------------------------------------------
// sendCommandAndRead()
// ---------------------------------------------------------------------------
string ElmoController::sendCommandAndRead(const string& cmd,
                                               int timeout_ms) {
    lock_guard<mutex> lock(port_mutex);

    if (!isConnected()) throw runtime_error("Serial port not open!");

    // --- Write command ---
    string full_cmd = cmd + "\r";
    ssize_t written = write(fd, full_cmd.c_str(), full_cmd.size());
    if (written < 0) {
        throw runtime_error(string("write() failed: ") +
                                 strerror(errno));
    }

    // Small delay so the drive can prepare its response, 500 Hz
    this_thread::sleep_for(chrono::milliseconds(2));

    // --- Read loop: discard echo, return the first numeric-looking line ---
    string response;
    auto start = chrono::steady_clock::now();

    while (chrono::duration_cast<chrono::milliseconds>(
               chrono::steady_clock::now() - start).count() < timeout_ms) {

        // Use select() to check readability with a short timeout
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        struct timeval tv;
        int remaining_ms =
            timeout_ms -
            (int)chrono::duration_cast<chrono::milliseconds>(
                chrono::steady_clock::now() - start).count();
        if (remaining_ms <= 0) break;
        tv.tv_sec  = remaining_ms / 1000;
        tv.tv_usec = (remaining_ms % 1000) * 1000;

        int ret = select(fd + 1, &rfds, nullptr, nullptr, &tv);
        if (ret <= 0) break; // timeout or error

        // Read one line (character by character)
        string line;
        char byte;
        while (chrono::duration_cast<chrono::milliseconds>(
                   chrono::steady_clock::now() - start).count() < timeout_ms) {

            // Per-byte select to avoid blocking forever mid-line
            FD_ZERO(&rfds);
            FD_SET(fd, &rfds);
            int rem2 =
                timeout_ms -
                (int)chrono::duration_cast<chrono::milliseconds>(
                    chrono::steady_clock::now() - start).count();
            if (rem2 <= 0) break;
            tv.tv_sec  = rem2 / 1000;
            tv.tv_usec = (rem2 % 1000) * 1000;

            if (select(fd + 1, &rfds, nullptr, nullptr, &tv) <= 0) break;

            ssize_t n = read(fd, &byte, 1);
            if (n <= 0) break;

            if (byte == '\r' || byte == '\n') {
                if (!line.empty()) break; // end of line
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

        // Strip ";CMD" suffix (Elmo echo marker)
        auto semicolon = line.find(';');
        if (semicolon != string::npos)
            line = line.substr(0, semicolon);

        // Accept if it starts with a digit, '-', or '.'
        if (!line.empty() &&
            (isdigit((unsigned char)line[0]) ||
             line[0] == '-' || line[0] == '.')) {
            response = line;
            break;
        }
        // Otherwise it's the echo — discard and continue
    }

    return response;
}

// ---------------------------------------------------------------------------
// Motor control
// ---------------------------------------------------------------------------
void ElmoController::motorOn() {
    try {
        cout << "Sending: Motor On (MO=1)" << endl;
        sendCommandAndRead("MO=1");
        wait(300);
    } catch (const exception& e) {
        cerr << "Error in motorOn: " << e.what() << endl;
        throw;
    }
}

void ElmoController::motorOff() {
    try {
        cout << "Sending: Motor Off (MO=0)" << endl;
        sendCommandAndRead("MO=0");
        wait(100);
    } catch (const exception& e) {
        cerr << "Error in motorOff: " << e.what() << endl;
        throw;
    }
}

void ElmoController::waitForMotionComplete(int timeout_ms) {
    cout << "Waiting for motion to complete..." << endl;
    auto start = chrono::steady_clock::now();

    while (chrono::duration_cast<chrono::milliseconds>(
               chrono::steady_clock::now() - start).count() < timeout_ms) {

        string response = sendCommandAndRead("MS");
        if (!response.empty()) {
            try {
                int status = stoi(response);
                if (status == 0) {
                    cout << "Motion complete (MS=0)." << endl;
                    return;
                } else if (status == 3) {
                    cout << "Warning: motor disabled (MS=3)." << endl;
                    return;
                }
                cout << "MS=" << status << endl;
            } catch (...) {}
        }
        this_thread::sleep_for(chrono::milliseconds(100));
    }
    cout << "Warning: motion timeout - check if limit switch was hit!" << endl;
}

// ---------------------------------------------------------------------------
// Mode setting
// ---------------------------------------------------------------------------
void ElmoController::setVelocityMode() {
    try {
        cout << "Sending: Set Velocity Mode (UM=2)" << endl;
        sendCommandAndRead("UM=2");
        wait(50);
    } catch (const exception& e) {
        cerr << "Error in setVelocityMode: " << e.what() << endl;
        throw;
    }
}

void ElmoController::setPositionMode() {
    try {
        cout << "Sending: Set Position Mode (UM=5)" << endl;
        sendCommandAndRead("UM=5");
        wait(50);
    } catch (const exception& e) {
        cerr << "Error in setPositionMode: " << e.what() << endl;
        throw;
    }
}

void ElmoController::setCurrentMode() {
    try {
        cout << "Sending: Set Current/Torque Mode (UM=1)" << endl;
        sendCommandAndRead("UM=1");
        wait(50);
    } catch (const exception& e) {
        cerr << "Error in setCurrentMode: " << e.what() << endl;
        throw;
    }
}

// ---------------------------------------------------------------------------
// Motion commands
// ---------------------------------------------------------------------------
void ElmoController::setVelocity(int32_t velocity) {
    try {
        string cmd = "JV=" + to_string(velocity);
        cout << "Sending: " << cmd << endl;
        sendCommandAndRead(cmd);
        wait(50);
        beginMotion(); 
    } catch (const exception& e) {
        cerr << "Error in setVelocity: " << e.what() << endl;
        throw;
    }
}

void ElmoController::setCurrent(float current) {
    try {
        string cmd = "TC=" + to_string(current);
        cout << "Sending: " << cmd << endl;
        sendCommandAndRead(cmd);
        wait(50);
        beginMotion(); 
    } catch (const exception& e) {
        cerr << "Error in setVelocity: " << e.what() << endl;
        throw;
    }
}

void ElmoController::setPosition(int32_t position) {
    try {
        string cmd = "PA=" + to_string(position);
        cout << "Sending: " << cmd << endl;
        sendCommandAndRead(cmd);
        wait(50);
    } catch (const exception& e) {
        cerr << "Error in setPosition: " << e.what() << endl;
        throw;
    }
}

void ElmoController::setPositionRelative(int32_t delta) {
    try {
        string cmd = "PR=" + to_string(delta);
        cout << "Sending: " << cmd << endl;
        sendCommandAndRead(cmd);
        wait(50);
    } catch (const exception& e) {
        cerr << "Error in setPositionRelative: " << e.what() << endl;
        throw;
    }
}

void ElmoController::beginMotion() {
    try {
        cout << "Sending: Begin Motion (BG)" << endl;
        sendCommandAndRead("BG");
        wait(50);
    } catch (const exception& e) {
        cerr << "Error in beginMotion: " << e.what() << endl;
        throw;
    }
}

void ElmoController::stopMotion() {
    try {
        cout << "Sending: Stop Motion (ST)" << endl;
        sendCommandAndRead("ST");
        wait(50);
    } catch (const exception& e) {
        cerr << "Error in stopMotion: " << e.what() << endl;
        throw;
    }
}

void ElmoController::velocityForTime(int32_t velocity, int duration_ms, int poll_ms) {
    try {
        cout << "Jogging at " << velocity
             << " counts/sec for " << duration_ms << "ms" << endl;

        setVelocity(velocity);

        auto start = chrono::steady_clock::now();
        while (chrono::duration_cast<chrono::milliseconds>(
                   chrono::steady_clock::now() - start).count() < duration_ms) {

            cout << "  POS= " << getPosition() << endl;
            cout << "  VEL= " << getVelocity() << endl;
            cout << "  CUR= " << getCurrent() << endl;
            wait(poll_ms);
        }

        stopMotion();
        wait(200);

        cout << "Jog complete. Final position: "
             << getPosition() << " counts" << endl;

    } catch (const exception& e) {
        cerr << "Error in jogForTime: " << e.what() << endl;
        stopMotion();
        throw;
    }
}

void ElmoController::currentForTime(float current, int duration_ms, int poll_ms) {
        try {
        cout << "Jogging at " << current
             << " A for " << duration_ms << "ms" << endl;

        setCurrent(current);

        auto start = chrono::steady_clock::now();
        while (chrono::duration_cast<chrono::milliseconds>(
                   chrono::steady_clock::now() - start).count() < duration_ms) {

            cout << "  POS= " << getPosition() << endl;
            cout << "  VEL= " << getVelocity() << endl;
            cout << "  CUR= " << getCurrent() << endl;
            wait(poll_ms);
        }

        stopMotion();
        wait(200);

        cout << "Jog complete. Final position: "
             << getPosition() << " counts" << endl;

    } catch (const exception& e) {
        cerr << "Error in jogForTime: " << e.what() << endl;
        stopMotion();
        throw;
    }
}
// ---------------------------------------------------------------------------
// Queries
// ---------------------------------------------------------------------------
int32_t ElmoController::getPosition() {
    try {
        // cout << "Requesting: Position (PX)" << endl;
        string response = sendCommandAndRead("PX");
        // cout << "Raw response: [" << response << "]" << endl;
        return stoi(response);
    } catch (const exception& e) {
        cerr << "Error in getPosition: " << e.what() << endl;
        return 0;
    }
}

int32_t ElmoController::getVelocity() {
    try {
        // cout << "Requesting: Velocity (VX)" << endl;
        string response = sendCommandAndRead("VX");
        return stoi(response);
    } catch (const exception& e) {
        cerr << "Error in getVelocity: " << e.what() << endl;
        return 0;
    }
}

float ElmoController::getCurrent() {
    try {
        // cout << "Requesting: Active Current (IQ)" << endl;
        string response = sendCommandAndRead("IQ");
        return stof(response);
    } catch (const exception& e) {
        cerr << "Error in getCurrent: " << e.what() << endl;
        return 0.0f;
    }
}

int32_t ElmoController::getStatus() {
    try {
        // cout << "Requesting: Status Register (SR)" << endl;
        string response = sendCommandAndRead("SR");
        return stoi(response);
    } catch (const exception& e) {
        cerr << "Error in getStatus: " << e.what() << endl;
        return 0;
    }
}

float ElmoController::getElmoTemperature() {
    try {
        string response = sendCommandAndRead("TI[1]"); // Temp in C
        return stof(response);
    } catch (const exception& e) {
        cerr << "Error in getElmoTemperature: " << e.what() << endl;
        return -999.0f;
    }
}

// ---------------------------------------------------------------------------
// Utilities
// ---------------------------------------------------------------------------
void ElmoController::wait(int milliseconds) {
    this_thread::sleep_for(chrono::milliseconds(milliseconds));
}

void ElmoController::sendRawCommand(const string& cmd) {
    lock_guard<mutex> lock(port_mutex);
    if (!isConnected()) throw runtime_error("Serial port not open!");
    string full_cmd = cmd + "\r";
    ssize_t written = write(fd, full_cmd.c_str(), full_cmd.size());
    if (written < 0) {
        throw runtime_error(string("write() failed: ") +
                                 strerror(errno));
    }
    cout << "Sent raw command: " << cmd << endl;
}

string ElmoController::readRawResponse() {
    lock_guard<mutex> lock(port_mutex);
    if (!isConnected()) throw runtime_error("Serial port not open!");
    char buffer[256];
    ssize_t n = read(fd, buffer, sizeof(buffer) - 1);
    if (n > 0) return string(buffer, n);
    return {};
}

string ElmoController::getSerialNumber() {
    try {
        string response = sendCommandAndRead("SN[4]");
        return response;
    } catch (const exception& e) {
        cerr << "Error in getSerialNumber: " << e.what() << endl;
        return "";
    }
}