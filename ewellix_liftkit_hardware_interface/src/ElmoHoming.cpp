#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <chrono>
#include <regex>
#include <cstdlib>
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

    string snA = ctrlA.getSerialNumber();
    string snB = ctrlB.getSerialNumber();

    // STRIP SEMICOLONS from serial numbers
    auto strip_semicolon = [](string& sn) {
        size_t pos = sn.find(';');
        if (pos != string::npos) {
            sn = sn.substr(0, pos);
        }
    };
    
    strip_semicolon(snA);
    strip_semicolon(snB);

    cout << "ACM2: " << snA << " = " << ElmoMap.at(snA) << endl;
    cout << "ACM3: " << snB << " = " << ElmoMap.at(snB) << endl;

    return (ElmoMap.at(snA) == "topMotor") ? make_pair(ref(ctrlA), ref(ctrlB)) : make_pair(ref(ctrlB), ref(ctrlA));
}

pair<string, string> loadPortsFromURDF(const string& urdf_file) {
    ifstream file(urdf_file);
    if (!file.is_open()) {
        throw runtime_error("Could not open URDF file: " + urdf_file);
    }
    
    string content((istreambuf_iterator<char>(file)), istreambuf_iterator<char>());
    file.close();
    
    // Extract all ports (any xacro:arg with "com_port" in name)
    regex port_regex(R"(<xacro:arg\s+name="[^"]*com_port[^"]*"\s+default="([^"]+))");
    
    smatch match;
    string port_top, port_bottom;
    string::const_iterator searchStart(content.cbegin());
    int port_count = 0;
    
    while (regex_search(searchStart, content.cend(), match, port_regex)) {
        if (port_count == 0) {
            port_top = match[1];
        } else if (port_count == 1) {
            port_bottom = match[1];
            break;
        }
        port_count++;
        searchStart = match.suffix().first;
    }
    
    cout << "Loaded from URDF:" << endl;
    cout << "  port_top (first): " << port_top << endl;
    cout << "  port_bottom (second): " << port_bottom << endl;
    
    return {port_top, port_bottom};
}

void crawlUntilStop(ElmoController& controller, const string& label,
                    int32_t speed, bool& success, int32_t& final_ticks) {
    try {
        controller.setVelocityMode();
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
    try {
        // Get HOME directory for portable paths
        const char* home = getenv("HOME");
        if (home == nullptr) {
            throw runtime_error("Could not determine HOME directory");
        }
        string home_str(home);
        
        string urdf_path = home_str + "/ewellix_lift_kit/ewellix_liftkit_description/urdf/parameters.xacro";
        string params_path = home_str + "/ewellix_lift_kit/ewellix_liftkit_deploy/config/ewellix_liftkit_parameters.yaml";
        
        // Load ports from URDF
        auto [port_top, port_bottom] = loadPortsFromURDF(urdf_path);
        
        ElmoController ctrlA(port_top, 115200);
        ElmoController ctrlB(port_bottom, 115200);

        printHeader("Connecting Motors");
        auto [elmoTop, elmoBot] = assignPorts(ctrlA, ctrlB);

        // ===== CALIBRATE DOWN =====
        printHeader("Calibrating DOWN");
        bool topSuccess_down = false, botSuccess_down = false;
        int32_t topTicks_down = 0, botTicks_down = 0;
        
        crawlUntilStop(elmoTop, "topMotor", -30, topSuccess_down, topTicks_down);
        elmoTop.motorOff();
        elmoTop.wait(1000);  // Pause between directions
        
        crawlUntilStop(elmoBot, "bottomMotor", -30, botSuccess_down, botTicks_down);
        elmoBot.motorOff();
        
        cout << "\n=== DOWN Results ===" << endl;
        cout << "Top Motor:    " << (topSuccess_down ? "OK" : "FAILED") << " - encoder zeroed" << endl;
        cout << "Bottom Motor: " << (botSuccess_down ? "OK" : "FAILED") << " - encoder zeroed" << endl;
        
        cout << "\nEnter minimum height in meters: ";
        double min_height_m;
        cin >> min_height_m;
        
        // ===== CALIBRATE UP =====
        printHeader("Calibrating UP");
        bool topSuccess_up = false, botSuccess_up = false;
        int32_t topTicks_up = 0, botTicks_up = 0;
        
        crawlUntilStop(elmoTop, "topMotor", 30, topSuccess_up, topTicks_up);
        elmoTop.motorOff();
        elmoTop.wait(1000);
        
        crawlUntilStop(elmoBot, "bottomMotor", 30, botSuccess_up, botTicks_up);
        elmoBot.motorOff();
        
        cout << "\n=== UP Results ===" << endl;
        cout << "Top Motor:    " << (topSuccess_up ? "OK" : "FAILED") << " - " << topTicks_up << " ticks" << endl;
        cout << "Bottom Motor: " << (botSuccess_up ? "OK" : "FAILED") << " - " << botTicks_up << " ticks" << endl;
        
        cout << "\nEnter maximum height in meters: ";
        double max_height_m;
        cin >> max_height_m;

        printHeader("Saving Calibration");
        
        // ===== UPDATE BOTH FILES ONCE =====
        
        // Update parameters YAML file
        ifstream params_in(params_path);
        string params_content((istreambuf_iterator<char>(params_in)), istreambuf_iterator<char>());
        params_in.close();
        
        params_content = regex_replace(params_content, 
            regex(R"(max_ticks_mot_1:\s*\d+)"),
            "max_ticks_mot_1: " + to_string(topTicks_up));
        params_content = regex_replace(params_content, 
            regex(R"(max_ticks_mot_2:\s*\d+)"),
            "max_ticks_mot_2: " + to_string(botTicks_up));
        params_content = regex_replace(params_content, 
            regex(R"(max_height_m:\s*[\d.]+)"),
            "max_height_m: " + to_string(max_height_m));
        params_content = regex_replace(params_content, 
            regex(R"(min_height_m:\s*[\d.]+)"),
            "min_height_m: " + to_string(min_height_m));
        
        ofstream params_out(params_path);
        if (params_out.is_open()) {
            params_out << params_content;
            cout << "Updated " << params_path << endl;
            params_out.close();
        }
        
        // Update xacro file
        ifstream xacro_in(urdf_path);
        string xacro_content((istreambuf_iterator<char>(xacro_in)), istreambuf_iterator<char>());
        xacro_in.close();
        
        xacro_content = regex_replace(xacro_content, 
            regex(R"(max_ticks_mot_1" default="[^"]+)"),
            "max_ticks_mot_1\" default=\"" + to_string(topTicks_up));
        xacro_content = regex_replace(xacro_content, 
            regex(R"(max_ticks_mot_2" default="[^"]+)"),
            "max_ticks_mot_2\" default=\"" + to_string(botTicks_up));
        xacro_content = regex_replace(xacro_content, 
            regex(R"(max_height_m" default="[^"]+)"),
            "max_height_m\" default=\"" + to_string(max_height_m));
        xacro_content = regex_replace(xacro_content, 
            regex(R"(min_height_m" default="[^"]+)"),
            "min_height_m\" default=\"" + to_string(min_height_m));
        
        ofstream xacro_out(urdf_path);
        if (xacro_out.is_open()) {
            xacro_out << xacro_content;
            cout << "Updated " << urdf_path << endl;
            xacro_out.close();
        }

        elmoTop.motorOff();
        elmoBot.motorOff();
        elmoTop.disconnect();
        elmoBot.disconnect();
        
        cout << "\n=== Full Calibration Complete ===" << endl;

    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return -1;
    }

    return 0;
}