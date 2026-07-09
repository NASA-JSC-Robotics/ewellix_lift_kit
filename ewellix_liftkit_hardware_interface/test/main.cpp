#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include "liftkit_hardware_interface/ElmoController.h"

using namespace std;

void printHeader(const string& title) {
    cout << "\n" << string(60, '=') << endl;
    cout << "  " << title << endl;
    cout << string(60, '=') << "\n" << endl;
}

const map<string, string> ElmoMap = {
    {"20210922", "bottomMotor"},
    {"20210926", "topMotor"}
};

pair<ElmoController&, ElmoController&> assignPorts(ElmoController& ctrlA, ElmoController& ctrlB) {
    ctrlA.connect();
    ctrlB.connect();
    ctrlA.wait(500);

    string snA = ctrlA.getSerialNumber();
    string snB = ctrlB.getSerialNumber();

    cout << "ACM2 SN: " << snA << " = " << ElmoMap.at(snA) << endl;
    cout << "ACM3 SN: " << snB << " = " << ElmoMap.at(snB) << endl;

    if (ElmoMap.at(snA) == "topMotor") {
        return {ctrlA, ctrlB};  // ctrlA is top, ctrlB is bot
    } else {
        return {ctrlB, ctrlA};  // ctrlB is top, ctrlA is bot
    }
}

int main() {
    try {
        ElmoController controllerA("/dev/ttyACM2", 115200);
        ElmoController controllerB("/dev/ttyACM3", 115200);

        auto [elmoTop, elmoBot] = assignPorts(controllerA, controllerB);

        printHeader("ELMO GOLD SOLO TWITTER - USB DEMO");
        
        // Step 1: Connect
        printHeader("Step 1: Connecting to Elmo Drive");
        elmoBot.connect();
        cout << elmoBot.getSerialNumber() << endl;
        elmoTop.connect();
        cout << elmoTop.getSerialNumber() << endl;
        elmoTop.wait(500);


        /*
        // Step 2: Set velocity mode
        printHeader("Step 2: Setting Velocity Mode");
        elmoTop.setVelocityMode();
        elmoBot.setVelocityMode();
        elmoTop.wait(100);
        */
        
        // Step 2: Set position mode
        printHeader("Step 2: Setting Position Mode");
        elmoBot.setPositionMode();
        elmoTop.setPositionMode();
        
        /*
        // Step 2: Set current mode
        printHeader("Step 2: Setting Current Mode");
        elmoBot.setCurrentMode();
        elmoTop.setCurrentMode();
        */
        
        elmoTop.wait(10);
        elmoTop.sendRawCommand("AC=100");
        elmoBot.sendRawCommand("AC=100");

        elmoTop.sendRawCommand("DC=100");
        elmoBot.sendRawCommand("DC=100");

        elmoTop.sendRawCommand("SD=100");
        elmoBot.sendRawCommand("SD=100");

        elmoTop.sendRawCommand("SP=30");
        elmoBot.sendRawCommand("SP=30");
        elmoTop.wait(50);

        // Step 3: Motor on
        printHeader("Step 3: Motors On");
        elmoTop.motorOn();
        elmoBot.motorOn();
        elmoTop.wait(500);
        
        // Step 4: Initial status
        printHeader("Step 4: Top Motor Initial Status");
        cout << "Current Position: " << elmoTop.getPosition() << " counts" << endl;
        cout << "Current Velocity: " << elmoTop.getVelocity() << " counts/sec" << endl;
        cout << "Current Current:  " << elmoTop.getCurrent() << " A" << endl;
        cout << "Initial Elmo Temperature: " << elmoTop.getElmoTemperature() << " C" << endl;
        printHeader("Step 4: Bottom Motor Initial Status");
        cout << "Current Position: " << elmoBot.getPosition() << " counts" << endl;
        cout << "Current Velocity: " << elmoBot.getVelocity() << " counts/sec" << endl;
        cout << "Current Current:  " << elmoBot.getCurrent() << " A" << endl;
        cout << "Initial Elmo Temperature: " << elmoBot.getElmoTemperature() << " C" << endl;
        
        
        // Step 5: Move forward
        printHeader("Step 5: Relative Move Forward");
        elmoTop.setPositionRelative(400);
        elmoBot.setPositionRelative(400);
        elmoTop.beginMotion();
        elmoBot.beginMotion();
        elmoTop.waitForMotionComplete(5000);
        elmoBot.waitForMotionComplete(5000);
        cout << "Top Motor Position after forward move: " << elmoTop.getPosition() << " counts" << endl;
        cout << "Bottom Motor Position after forward move: " << elmoBot.getPosition() << " counts" << endl;
        
        // Step 6: Move backward
        printHeader("Step 6: Relative Move Backward");
        elmoTop.setPositionRelative(-400);
        elmoBot.setPositionRelative(-400);
        elmoTop.beginMotion();
        elmoBot.beginMotion();
        elmoTop.waitForMotionComplete(5000);
        elmoBot.waitForMotionComplete(5000);
        cout << "Top Motor Position after backwards move: " << elmoTop.getPosition() << " counts" << endl;
        cout << "Bottom Motor Position after backwards move: " << elmoBot.getPosition() << " counts" << endl;
        
        // Step 5: Move forward
        printHeader("Step 5: Relative Move Forward");
        elmoTop.setPositionRelative(200);
        elmoBot.setPositionRelative(200);
        elmoTop.beginMotion();
        elmoBot.beginMotion();
        elmoTop.waitForMotionComplete(5000);
        elmoBot.waitForMotionComplete(5000);
        cout << "Top Motor Position after forward move: " << elmoTop.getPosition() << " counts" << endl;
        cout << "Bottom Motor Position after forward move: " << elmoBot.getPosition() << " counts" << endl;
        
        // Step 6: Move backward
        printHeader("Step 6: Relative Move Backward");
        elmoTop.setPositionRelative(-200);
        elmoBot.setPositionRelative(-200);
        elmoTop.beginMotion();
        elmoBot.beginMotion();
        elmoTop.waitForMotionComplete(5000);
        elmoBot.waitForMotionComplete(5000);
        cout << "Top Motor Position after backwards move: " << elmoTop.getPosition() << " counts" << endl;
        cout << "Bottom Motor Position after backwards move: " << elmoBot.getPosition() << " counts" << endl;
        

        /*
        // Step 5: Jog forward
        printHeader("Step 5: Jog Forward");
        cout << "[TOP]" << endl;
        elmoTop.velocityForTime(100, 2000);
        elmoTop.wait(500);
        cout << "[BOT]" << endl;
        elmoBot.velocityForTime(100, 2000);
        elmoTop.wait(500);

        // Step 6: Jog backward
        printHeader("Step 6: Jog Backward");
        cout << "[TOP]" << endl;
        elmoTop.velocityForTime(-100, 1800);
        elmoTop.wait(500);
        cout << "[BOT]" << endl;
        elmoBot.velocityForTime(-100, 1800);
        elmoTop.wait(500);
        */
        /*
        // Step 5: Jog forward
        printHeader("Step 5: Jog Forward");
        cout << "[TOP]" << endl;
        elmoTop.currentForTime(1.5, 4000);
        elmoTop.wait(500);
        cout << "[BOT]" << endl;
        elmoBot.currentForTime(1.5, 4000);
        elmoTop.wait(500);

        // Step 6: Jog backward
        printHeader("Step 6: Jog Backward");
        cout << "[TOP]" << endl;
        elmoTop.currentForTime(-1.5, 1800);
        elmoTop.wait(500);
        cout << "[BOT]" << endl;
        elmoBot.currentForTime(-1.5, 1800);
        elmoTop.wait(500);
        */ 
        // Step 7: Motor off
        printHeader("Step 7: Motor Off");
        elmoTop.stopMotion();
        elmoBot.stopMotion();
        elmoTop.wait(200);
        elmoTop.motorOff();
        elmoBot.motorOff();
        elmoTop.wait(500);
        
        // Step 8: Final status
        printHeader("Step 8: Final Status");
        cout << "Top Motor Final Position: " << elmoTop.getPosition() << " counts" << endl;
        cout << "Top Motor Final Velocity: " << elmoTop.getVelocity() << " counts/sec" << endl;
        cout << "Top Elmo Final Temperature: " << elmoTop.getElmoTemperature() << " C" << endl;
        cout << "Bottom Motor Final Position: " << elmoBot.getPosition() << " counts" << endl;
        cout << "Bottom Motor Final Velocity: " << elmoBot.getVelocity() << " counts/sec" << endl;
        cout << "Bottom Elmo Final Temperature: " << elmoTop.getElmoTemperature() << " C" << endl;

        printHeader("Disconnecting");
        elmoTop.disconnect();
        elmoBot.disconnect();
        
        cout << "\n Demo completed successfully!\n" << endl;
        
    } catch (const exception& e) {
        cerr << "\n Error: " << e.what() << endl;
        return -1;
    }
    
    return 0;
}