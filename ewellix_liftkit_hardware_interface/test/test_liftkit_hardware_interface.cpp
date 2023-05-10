#include "../include/liftkit_hardware_interface/serial_com_tlt.h"

using namespace std;
int main()
{
    std::string port = "/dev/ttyUSB0";
    int baudrate = 38400;
    SerialComTlt srl_;
    thread com_thread_;

    if(srl_.startSerialCom(port,baudrate)){

        com_thread_ = thread(&SerialComTlt::comLoop,&srl_); //  RC thread
        if(srl_.startRs232Com()){        // Com started
            srl_.moveUp();
            // srl_.setColumnSize(0.2);
            sleep(2);
        }
        else{
            cout << "TltNode::TltNode - Remote function activation : Fail! " << endl;
            abort();
        }
    }
    else{
        cout << "TltNode::TltNode - Serial Com : Fail! " << endl;
        abort();
    }

    srl_.stopRs232Com();
    srl_.run_= false;       // stop RC thread loop
    com_thread_.join();

    return 0;
}
