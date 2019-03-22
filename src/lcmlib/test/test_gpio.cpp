#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <string>
#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include "GPIOClass.hpp"

using namespace std;

int main (void)
{

    string inputstate;
    GPIOClass *gpio20 = new GPIOClass("20");
    GPIOClass *gpio21 = new GPIOClass("21");
    GPIOClass *gpio6 = new GPIOClass("6");
    GPIOClass *gpio13 = new GPIOClass("13");
    gpio20->export_gpio();
    gpio21->export_gpio();
    gpio6->export_gpio();
    gpio13->export_gpio();
    cout << " GPIO pins exported" << endl;
    gpio20->setdir_gpio("out");
    gpio21->setdir_gpio("out");
    gpio6->setdir_gpio("out");
    gpio13->setdir_gpio("out");
    gpio20->setval_gpio("1");
    gpio13->setval_gpio("1");
    gpio20->setval_gpio("0");
    gpio13->setval_gpio("0");
    usleep(1000000);                  // wait for 0.5 seconds
    return 0;
}