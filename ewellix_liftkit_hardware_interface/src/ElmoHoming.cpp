#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <chrono>
#include "liftkit_hardware_interface/ElmoController.h"

using namespace std;

const map<string, string> ElmoMap = {
    {"20210922", "bottomMotor"},
    {"20210926", "topMotor"}
};

void printHeader(const string& title) {
    cout << "\n=== " << title << " ===" << endl;
}

pair<ElmoController&, ElmoController&> assignPorts(ElmoController& ctrlA, ElmoController& ctrlB) {
    ctrlA.connect();
    ctrlB.connect();
    ctrlA.wait(100);

    string snA = ctrlA.getSerialNumber();
    string snB = ctrlB.getSerialNumber();

    cout << "ACM0: " << snA << " = " << ElmoMap.at(snA) << endl;
    cout << "ACM1: " << snB << " = " << ElmoMap.at(snB) << endl;

    return (ElmoMap.at(snA) == "topMotor") ? make_pair(ref(ctrlA), ref(ctrlB)) 
                                           : make_pair(ref(ctrlB), ref(ctrlA));
}

void crawlUntilStop(ElmoController& controller, const string& label,
                    int32_t speed, bool& success, int32_t& final_ticks) {
    try {
        controller.setVelocityMode();
        controller.wait(100);
        controller.motorOn();
        controller.wait(500);

        cout << "[" << label << "] Moving..." << endl;
        controller.setVelocity(speed);
        
        int32_t last_pos = controller.getPosition();
        int consec_vel_stall = 0;
        int consec_pos_stall = 0;
        
        auto start = chrono::steady_clock::now();
        
        while (chrono::duration_cast<chrono::milliseconds>(
                   chrono::steady_clock::now() - start).count() < 35000) {
            
            int32_t vel = controller.getVelocity();
            int32_t pos = controller.getPosition();
            
            cout << "[" << label << "] VEL=" << vel << " POS=" << pos << endl;
            
            // Velocity stall detection
            if (abs(vel) < 5) {
                consec_vel_stall++;
            } else {
                consec_vel_stall = 0;
            }
            
            // Position stall detection
            if (pos == last_pos) {
                consec_pos_stall++;
            } else {
                consec_pos_stall = 0;
            }
            last_pos = pos;
            
            // Hard stop = both velocity AND position stalled
            if (consec_vel_stall >= 3 && consec_pos_stall >= 5) {
                cout << "[" << label << "] Hard stop confirmed!" << endl;
                controller.stopMotion();
                controller.wait(100);
                
                final_ticks = pos;
                success = true;
                
                // Zero encoder at down position
                if (speed < 0) {
                    controller.zeroPosition();
                    cout << "[" << label << "] Encoder zeroed." << endl;
                }
                return;
            }
            
            controller.wait(500);
        }
        
        cout << "[" << label << "] TIMEOUT!" << endl;
        controller.stopMotion();
        final_ticks = controller.getPosition();
        success = false;
        
    } catch (const exception& e) {
        cerr << "[" << label << "] ERROR: " << e.what() << endl;
        controller.stopMotion();
        success = false;
    }
}

int main(int argc, char** argv) {
    if (argc < 2 || (string(argv[1]) != "up" && string(argv[1]) != "down")) {
        cerr << "Usage: " << argv[0] << " <up|down>" << endl;
        return -1;
    }

    string direction = argv[1];
    bool is_up = (direction == "up");
    int32_t speed = is_up ? 50 : -50;

    try {
        ElmoController::setCalibrationMode(true);
        ElmoController ctrlA("/dev/ttyACM0", 115200);
        ElmoController ctrlB("/dev/ttyACM1", 115200);

        printHeader("Connecting Motors");
        auto [elmoTop, elmoBot] = assignPorts(ctrlA, ctrlB);

        printHeader("Calibrating " + direction);
        bool topSuccess = false, botSuccess = false;
        int32_t topTicks = 0, botTicks = 0;

        // Run motors sequentially
        crawlUntilStop(elmoTop, "topMotor", speed, topSuccess, topTicks);
        elmoTop.motorOff();
        crawlUntilStop(elmoBot, "bottomMotor", speed, botSuccess, botTicks);
        elmoBot.motorOff();
        
        cout << "\n=== RESULTS ===" << endl;
        cout << "Top Motor:    " << (topSuccess ? "OK" : "FAILED") << " - " << topTicks << " ticks" << endl;
        cout << "Bottom Motor: " << (botSuccess ? "OK" : "FAILED") << " - " << botTicks << " ticks" << endl;

        cout << "\nEnter height in meters: ";
        double height_m;
        cin >> height_m;

        printHeader("Saving Calibration");
        ofstream f("elmo_calibration_" + direction + ".yaml");
        if (f.is_open()) {
            if (is_up) {
                f << "max_ticks_mot_1: " << topTicks << "\n"
                  << "max_ticks_mot_2: " << botTicks << "\n"
                  << "max_height_m: " << height_m << "\n";
            } else {
                f << "min_ticks_mot_1: " << topTicks << "\n"
                  << "min_ticks_mot_2: " << botTicks << "\n"
                  << "min_height_m: " << height_m << "\n";
            }
            cout << "Saved to elmo_calibration_" << direction << ".yaml" << endl;
        }

        elmoTop.motorOff();
        elmoBot.motorOff();
        elmoTop.disconnect();
        elmoBot.disconnect();

    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return -1;
    }

    return 0;
}