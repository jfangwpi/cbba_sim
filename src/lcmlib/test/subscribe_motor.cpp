#include <stdio.h>
#include "GPIOClass.hpp"
#include <chrono>
#include <thread>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/cbba_lcm_msgs.hpp"

class Handler
{    
    public: 
    ~Handler(){};

    GPIOClass* gpio20 = new GPIOClass("20");
    GPIOClass* gpio21 = new GPIOClass("21");
    GPIOClass* gpio6 = new GPIOClass("6");
    GPIOClass* gpio13 = new GPIOClass("13");


    public:

    void handleMessage(const lcm::ReceiveBuffer* rbuf,
                    const std::string& chan,
                    const cbba_lcm_msgs::example_t* msg){
                    gpio20->export_gpio();
                    gpio21->export_gpio();
                    gpio6->export_gpio();
                    gpio13->export_gpio();
                    gpio20->setdir_gpio("out");
                    gpio21->setdir_gpio("out");
                    gpio6->setdir_gpio("out");
                    gpio13->setdir_gpio("out");

                    std::string out_signal = std::to_string(msg->position[0]);
                    gpio13->setval_gpio(out_signal);
                    }
};


int main(int argc, char** argv){
    lcm::LCM lcm;
    if(!lcm.good()){
        return 1;
    }

    Handler handlerObject;
    lcm.subscribe("EXAMPLE", &Handler::handleMessage, &handlerObject);

    while(0 == lcm.handle());

    return 0;
    
}