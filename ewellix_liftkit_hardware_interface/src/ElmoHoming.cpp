#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include "liftkit_hardware_interface/ElmoController.h"

using namespace std;

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

    cout << "ACM0 SN: " << snA << " = " << ElmoMap.at(snA) << endl;
    cout << "ACM1 SN: " << snB << " = " << ElmoMap.at(snB) << endl;

    if (ElmoMap.at(snA) == "topMotor") {
        return {ctrlA, ctrlB};  // ctrlA is top, ctrlB is bot
    } else {
        return {ctrlB, ctrlA};  // ctrlB is top, ctrlA is bot
    }
}

int main() {
    try {
        ElmoController controllerA("/dev/ttyACM0", 115200);
        ElmoController controllerB("/dev/ttyACM1", 115200);

        auto [elmoTop, elmoBot] = assignPorts(controllerA, controllerB);
        
        // Step 1: Connect
        elmoBot.connect();
        cout << elmoBot.getSerialNumber() << endl;
        elmoTop.connect();
        cout << elmoTop.getSerialNumber() << endl;
        elmoTop.wait(500);
        
        // Step 2: Set position mode
        printHeader("Step 2: Setting Position Mode");
        elmoBot.setVelocityMode();
        elmoTop.setVelocityMode();
        
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