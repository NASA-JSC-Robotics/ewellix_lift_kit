// Copyright 2025 NASA Johnson Space Center
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "../include/liftkit_hardware_interface/serial_com_tlt.h"

using namespace std;
int main()
{
  std::string port = "/dev/ttyUSB0";
  int baudrate = 38400;
  SerialComTlt srl_;
  thread com_thread_;

  if (srl_.startSerialCom(port, baudrate))
  {
    com_thread_ = thread(&SerialComTlt::comLoop, &srl_);  //  RC thread
    if (srl_.startRs232Com())
    {  // Com started
      srl_.desired_pose_ = 0.0;
      sleep(5.0);
      srl_.desired_pose_ = 0.2;
      sleep(5.0);
      ;
    }
    else
    {
      cout << "TltNode::TltNode - Remote function activation : Fail! " << endl;
      abort();
    }
  }
  else
  {
    cout << "TltNode::TltNode - Serial Com : Fail! " << endl;
    abort();
  }

  srl_.stopRs232Com();
  srl_.run_ = false;  // stop RC thread loop
  com_thread_.join();

  return 0;
}
