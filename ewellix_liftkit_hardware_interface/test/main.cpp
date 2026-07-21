#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <cmath>
#include <chrono>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <csignal>
#include <atomic>
#include <cstdio>
#include <fstream>
#include <vector>
#include <numeric>
#include <algorithm>
#include "liftkit_hardware_interface/ElmoController.h"

using namespace std;

// === HARDCODED SINE WAVE PARAMETERS ===
const double CENTER = 0.35;                   // Center position (meters)
const double AMPLITUDE = 0.24;                // Amplitude above/below center (meters)
const double PERIOD_SECONDS = 40.0;            // 40 second period
const double INIT_DURATION_SECONDS = 10.0;     // 10 second initialization
const double UPDATE_FREQUENCY_HZ = 45.0;       // 45 Hz update rate
const double TEST_CYCLES = 3.0;                // Number of sine wave cycles to run
const bool ENABLE_LIVE_PLOT = true;            // Enable real-time gnuplot
// ======================================

// Global variables for signal handling
atomic<bool> shutdown_requested(false);
ElmoController* g_elmoTop = nullptr;
ElmoController* g_elmoBot = nullptr;
FILE* gnuplot_pipe = nullptr;

// Signal handler for Ctrl+C
void signalHandler(int signum) {
    cout << "\n\n[INTERRUPT] Ctrl+C pressed - initiating emergency stop..." << endl;
    shutdown_requested = true;
}

void printHeader(const string& title) {
    cout << "\n" << string(60, '=') << endl;
    cout << "  " << title << endl;
    cout << string(60, '=') << "\n" << endl;
}

struct SineWaveConfig {
    string com_port_top;
    string com_port_bottom;
    double min_height_m;
    double max_height_m;
    int max_ticks_mot_1;
    int max_ticks_mot_2;
};

SineWaveConfig loadConfigFromYAML(const string& yaml_path) {
    try {
        YAML::Node config = YAML::LoadFile(yaml_path);
        YAML::Node ewellix = config["ewellix_liftkit"];
        
        SineWaveConfig cfg;
        cfg.com_port_top = ewellix["com_port_top"].as<string>();
        cfg.com_port_bottom = ewellix["com_port_bottom"].as<string>();
        cfg.min_height_m = ewellix["min_height_m"].as<double>();
        cfg.max_height_m = ewellix["max_height_m"].as<double>();
        cfg.max_ticks_mot_1 = ewellix["max_ticks_mot_1"].as<int>();
        cfg.max_ticks_mot_2 = ewellix["max_ticks_mot_2"].as<int>();
        
        cout << "✓ Loaded configuration from: " << yaml_path << endl;
        cout << "  COM Port Top: " << cfg.com_port_top << endl;
        cout << "  COM Port Bottom: " << cfg.com_port_bottom << endl;
        cout << "  Min Height: " << fixed << setprecision(6) << cfg.min_height_m << " m" << endl;
        cout << "  Max Height: " << fixed << setprecision(6) << cfg.max_height_m << " m" << endl;
        
        return cfg;
    } catch (const exception& e) {
        cerr << "Error loading YAML: " << e.what() << endl;
        throw;
    }
}

void initGnuplot(double amplitude, double period, double center, 
                 double min_height, double max_height) {
    if (!ENABLE_LIVE_PLOT) return;
    
    gnuplot_pipe = popen("gnuplot -persistent", "w");
    if (!gnuplot_pipe) {
        cerr << "Warning: Could not open gnuplot pipe. Live plotting disabled." << endl;
        return;
    }
    
    // Configure gnuplot
    fprintf(gnuplot_pipe, "set title 'Sine Wave Motion Test - Command vs Actual Position'\n");
    fprintf(gnuplot_pipe, "set xlabel 'Time (s)'\n");
    fprintf(gnuplot_pipe, "set ylabel 'Position (m)'\n");
    fprintf(gnuplot_pipe, "set yrange [%.3f:%.3f]\n", min_height - 0.05, max_height + 0.05);
    fprintf(gnuplot_pipe, "set grid\n");
    fprintf(gnuplot_pipe, "set key top left\n");
    fprintf(gnuplot_pipe, "set style data lines\n");
    fprintf(gnuplot_pipe, "set linewidth 2\n");
    
    fflush(gnuplot_pipe);
    
    cout << "[PLOT] Gnuplot live plotting enabled." << endl;
}

void updateGnuplot(double elapsed, double cmd_pos, double actual_pos) {
    if (!gnuplot_pipe || !ENABLE_LIVE_PLOT) return;
    
    // Write data to temporary file
    ofstream data_file("/tmp/sine_wave_plot.dat", ios::app);
    data_file << fixed << setprecision(4) << elapsed << " " << cmd_pos << " " << actual_pos << "\n";
    data_file.close();
    fprintf(gnuplot_pipe, "plot '/tmp/sine_wave_plot.dat' using 1:2 with lines title ' Command' lw 2.5 lc rgb 'blue', ");
    fprintf(gnuplot_pipe, "'/tmp/sine_wave_plot.dat' using 1:3 with lines title 'Actual' lw 2.5 lc rgb 'red'\n");
    fflush(gnuplot_pipe);
}

void closeGnuplot() {
    if (gnuplot_pipe && ENABLE_LIVE_PLOT) {
        fprintf(gnuplot_pipe, "set title 'Sine Wave Motion Test - COMPLETE'\n");
        fflush(gnuplot_pipe);
        cout << "[PLOT] Plot window remains open. Close it manually." << endl;
    }
}

int main() {
    try {
        // Register signal handler for Ctrl+C
        signal(SIGINT, signalHandler);
        cout << "[INFO] Press Ctrl+C at any time to safely stop the motors and exit." << endl;
        
        // Load configuration from YAML
        printHeader("Loading Configuration");
        string yaml_path = "ewellix_liftkit_deploy/config/ewellix_liftkit_parameters.yaml"; 
        SineWaveConfig config = loadConfigFromYAML(yaml_path);
        
        // Create controllers with ports from YAML
        ElmoController elmoTop(config.com_port_top, 115200);
        ElmoController elmoBot(config.com_port_bottom, 115200);
        
        // Store global pointers for signal handler
        g_elmoTop = &elmoTop;
        g_elmoBot = &elmoBot;

        printHeader("Connecting to Elmo Drives");
        elmoTop.connect();
        elmoBot.connect();
        elmoTop.wait(500);

        printHeader("SINE WAVE MOTION TEST - ELMO GOLD SOLO");
        
        // Setup motors
        printHeader("Step 1: Setup Motors");
        elmoBot.setPositionMode();
        elmoTop.setPositionMode();
        
        elmoTop.wait(10);
        elmoTop.sendRawCommand("AC=100");
        elmoBot.sendRawCommand("AC=100");
        elmoTop.sendRawCommand("DC=100");
        elmoBot.sendRawCommand("DC=100");
        elmoTop.sendRawCommand("SD=100");
        elmoBot.sendRawCommand("SD=100");
        elmoTop.sendRawCommand("SP=100");
        elmoBot.sendRawCommand("SP=100");
        elmoTop.wait(50);
        
        // Motor on
        printHeader("Step 2: Motors On");
        elmoTop.motorOn();
        elmoBot.motorOn();
        elmoTop.wait(500);
        
        // Get current position (assume already homed)
        printHeader("Step 3: Current Motor Status");
        int32_t current_top = elmoTop.getPosition();
        int32_t current_bot = elmoBot.getPosition();
        int32_t current_total = current_top + current_bot;
        cout << "Top Motor Position: " << current_top << " counts" << endl;
        cout << "Bottom Motor Position: " << current_bot << " counts" << endl;
        cout << "Total Position: " << current_total << " counts" << endl;
        
        // === SINE WAVE PARAMETERS ===
        double min_height = config.min_height_m;
        double max_height = config.max_height_m;
        double center = CENTER;
        double amplitude = AMPLITUDE;
        
        double period = PERIOD_SECONDS;
        double init_duration = INIT_DURATION_SECONDS;
        double freq = UPDATE_FREQUENCY_HZ;
        double dt = 1.0 / freq;
        
        int max_ticks_total = config.max_ticks_mot_1 + config.max_ticks_mot_2;
        int max_ticks_bot = config.max_ticks_mot_1;
        int max_ticks_top = config.max_ticks_mot_2;
        double stroke_length = max_height - min_height;
        
        // =============================
        printHeader("Step 4: Sine Wave Motion Test");
        cout << "Sine Wave Parameters:" << endl;
        cout << "  Center: " << fixed << setprecision(3) << center << "m" << endl;
        cout << "  Amplitude: ±" << amplitude << "m" << endl;
        cout << "  Min Position: " << (center - amplitude) << "m" << endl;
        cout << "  Max Position: " << (center + amplitude) << "m" << endl;
        cout << "\nMotion Parameters:" << endl;
        cout << "  Period: " << period << "s" << endl;
        cout << "  Initialization Time: " << init_duration << "s" << endl;
        cout << "  Test Cycles: " << fixed << setprecision(1) << TEST_CYCLES << endl;
        cout << "  Total Duration: " << (init_duration + (TEST_CYCLES * period)) << "s" << endl;
        cout << "  Update Rate: " << freq << " Hz (dt=" << dt*1000 << "ms)" << endl;
        if (ENABLE_LIVE_PLOT) {
            cout << "  Live Plotting: ENABLED (watch the gnuplot window)" << endl;
        }
        cout << "\n" << endl;
        
        // Initialize live plotting with all parameters
        if (ENABLE_LIVE_PLOT) {
            system("rm -f /tmp/sine_wave_plot.dat");
            initGnuplot(amplitude, period, center, min_height, max_height);
        }
        
        auto start_time = chrono::high_resolution_clock::now();
        bool init_complete = false;
        auto sine_start_time = start_time;
        
        int32_t last_target_bottom = current_bot;
        int32_t last_target_top = current_top;
        
        // Track statistics for live display
        vector<double> error_history;
        const int max_history = 100;
        
        // === STREAM COMMANDS LIKE PYTHON VERSION ===
        while (!shutdown_requested) {
            auto current_time = chrono::high_resolution_clock::now();
            double elapsed = chrono::duration<double>(current_time - start_time).count();
            
            double position = center;
            string phase_info = "INIT";
            
            // Phase 1: Initialization - hold at center
            if (!init_complete) {
                if (elapsed >= init_duration) {
                    init_complete = true;
                    sine_start_time = current_time;
                    cout << "\n✓ Initialization complete. Starting sine wave motion.\n" << endl;
                } else {
                    position = center;
                    phase_info = "INIT";
                }
            }
            
            // Phase 2: Sine wave
            if (init_complete) {
                double sine_elapsed = chrono::duration<double>(current_time - sine_start_time).count();
                double angle = 2.0 * M_PI * sine_elapsed / period;
                position = center + amplitude * sin(angle);
                phase_info = "SINE";
            }
            
            // Clamp to safe range
            position = max(min_height, min(max_height, position));
            
            // Convert target height to total ticks needed
            int32_t desired_total_ticks = static_cast<int32_t>(
                (position - min_height) / stroke_length * max_ticks_total);
            
            // === HARDWARE INTERFACE SEQUENTIAL LOGIC ===
            // Read current state
            int32_t current_bottom = elmoBot.getPosition();
            int32_t current_top = elmoTop.getPosition();
            int32_t current_total = current_bottom + current_top;
            
            bool extending = (desired_total_ticks > current_total);
            bool retracting = (desired_total_ticks < current_total);
            
            int32_t target_bottom_ticks = 0;
            int32_t target_top_ticks = 0;
            
            if (extending) {
                // EXTEND: Bottom fills first, then top
                target_bottom_ticks = std::min(desired_total_ticks, max_ticks_bot);
                target_top_ticks = std::max(0, desired_total_ticks - max_ticks_bot);
                
                // Sequential: only move top when bottom is near max (99%)
                int32_t bottom_threshold = static_cast<int32_t>(max_ticks_bot * 0.99);
                if (current_bottom < bottom_threshold) {
                    target_top_ticks = current_top;  // Keep top stationary
                }
            }
            else if (retracting) {
                // RETRACT: Top retracts first, then bottom
                int32_t top_threshold = static_cast<int32_t>(max_ticks_top * 0.01);
                
                if (current_top > top_threshold) {
                    // Top still has extension, retract it first
                    target_top_ticks = std::max(0, desired_total_ticks - max_ticks_bot);
                    target_bottom_ticks = current_bottom;  // Keep bottom stationary
                }
                else {
                    // Top is retracted, now retract bottom
                    target_bottom_ticks = std::max(0, desired_total_ticks);
                    target_top_ticks = 0;
                }
            }
            else {
                // Not moving, maintain current positions
                target_bottom_ticks = current_bottom;
                target_top_ticks = current_top;
            }
            
            // Send commands only if targets changed (don't spam serial)
            if (target_bottom_ticks != last_target_bottom) {
                elmoBot.setPosition(target_bottom_ticks);
                elmoBot.beginMotion();
                last_target_bottom = target_bottom_ticks;
            }
            
            if (target_top_ticks != last_target_top) {
                elmoTop.setPosition(target_top_ticks);
                elmoTop.beginMotion();
                last_target_top = target_top_ticks;
            }
            
            // Get feedback WITHOUT blocking (like Python version)
            int32_t feedback_bottom = elmoBot.getPosition();
            int32_t feedback_top = elmoTop.getPosition();
            int32_t feedback_total = feedback_bottom + feedback_top;
            double feedback_height = (feedback_total / (double)max_ticks_total * stroke_length + min_height);
            double current_vel_top = elmoTop.getVelocity();
            double current_vel_bot = elmoBot.getVelocity();
            
            // Calculate error from commanded position
            double error = position - feedback_height;
            
            // Update error history for statistics
            error_history.push_back(abs(error));
            if (error_history.size() > max_history) {
                error_history.erase(error_history.begin());
            }
            
            // Calculate statistics
            double avg_error = 0.0, max_error = 0.0;
            if (!error_history.empty()) {
                avg_error = accumulate(error_history.begin(), error_history.end(), 0.0) / error_history.size();
                max_error = *max_element(error_history.begin(), error_history.end());
            }
            
            // Log output (like Python version)
            cout << "[" << phase_info << "] t=" << fixed << setprecision(2) 
                 << setw(6) << elapsed << "s | Cmd: " << setprecision(4) << position << "m | "
                 << "Actual: " << feedback_height << "m | "
                 << "Error: " << showpos << error << noshowpos << "m | "
                 << "Bot Vel: " << current_vel_bot << " cnt/s | "
                 << "Top Vel: " << current_vel_top << " cnt/s" << endl;
            
            // Update live plot with statistics
            if (phase_info == "SINE") {
                updateGnuplot(elapsed, position, feedback_height);
            }
            
            // Exit condition: Run for specified number of sine cycles
            if (elapsed > (init_duration + (TEST_CYCLES * period))) {
                cout << "\n✓ Test duration complete." << endl;
                break;
            }
            
            // Sleep at update frequency rate
            this_thread::sleep_for(chrono::duration<double>(dt));
        }
        
        // Cleanup after shutdown (runs whether from Ctrl+C or natural exit)
        printHeader("Step 5: Motor Off");
        cout << "[CLEANUP] Stopping motion..." << endl;
        elmoTop.stopMotion();
        elmoBot.stopMotion();
        elmoTop.wait(200);
        
        cout << "[CLEANUP] Disabling motors..." << endl;
        elmoTop.motorOff();
        elmoBot.motorOff();
        elmoTop.wait(500);
        
        cout << "[CLEANUP] Motors safely disabled." << endl;
        
        // Final status
        printHeader("Step 6: Final Status");
        int32_t final_bottom = elmoBot.getPosition();
        int32_t final_top = elmoTop.getPosition();
        int32_t final_total = final_bottom + final_top;
        double final_height = (final_total / (double)max_ticks_total * stroke_length + min_height);
        
        cout << "Top Motor Final Position: " << final_top << " counts" << endl;
        cout << "Bottom Motor Final Position: " << final_bottom << " counts" << endl;
        cout << "Final Height: " << final_height << " m" << endl;
        cout << "Top Motor Final Temperature: " << elmoTop.getElmoTemperature() << " C" << endl;
        cout << "Bottom Motor Final Temperature: " << elmoBot.getElmoTemperature() << " C" << endl;
        
        // === PERFORMANCE SUMMARY ===
        printHeader("Step 7: Performance Summary");
        if (!error_history.empty()) {
            double final_avg_error = accumulate(error_history.begin(), error_history.end(), 0.0) / error_history.size();
            double final_max_error = *max_element(error_history.begin(), error_history.end());
            double final_min_error = *min_element(error_history.begin(), error_history.end());
            
            // Calculate standard deviation
            double mean = final_avg_error;
            double variance = 0.0;
            for (double e : error_history) {
                variance += (e - mean) * (e - mean);
            }
            variance /= error_history.size();
            double std_dev = sqrt(variance);
            
            cout << "Average Tracking Error:  " << fixed << setprecision(6) << final_avg_error << " m ("
                 << final_avg_error * 1000 << " mm)" << endl;
            cout << "Maximum Tracking Error:  " << final_max_error << " m ("
                 << final_max_error * 1000 << " mm)" << endl;
            cout << "Minimum Tracking Error:  " << final_min_error << " m ("
                 << final_min_error * 1000 << " mm)" << endl;
            cout << "Error Standard Deviation: " << std_dev << " m ("
                 << std_dev * 1000 << " mm)" << endl;
        }

        printHeader("Disconnecting");
        elmoTop.disconnect();
        elmoBot.disconnect();
        
        // Close gnuplot
        closeGnuplot();
        
        // Clear global pointers
        g_elmoTop = nullptr;
        g_elmoBot = nullptr;
        
        cout << "\n Sine wave test completed successfully!\n" << endl;
        
    } catch (const exception& e) {
        cerr << "\n Error: " << e.what() << endl;
        closeGnuplot();
        return -1;
    }
    
    return 0;
}