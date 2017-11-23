#include <vector>
#include <iostream>
#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    isFirstMsmt = true;

    nCTEValues = 20;
    cteSum = 0.0;
    prev_cte = 0.0;

    std::cout << "p, i, d kps = " << this->Kp << ", " << this->Ki << ", " << this->Kd << std::endl;
}


void PID::UpdateError(double cte) {

    if (cteList.size() > 0){
        prev_cte = cteList.back();
    }
    if (cteList.size() >= nCTEValues){
        //If the list grows more than max allowed, remove the first element
        //i.e keep the lat 20 from the list
        cteList.erase(cteList.begin());
        cteSum -= cteList.front();
    }
    cteList.push_back(cte);
    cteSum += cte;
    p_error = cte;
    d_error = cte - prev_cte;
    i_error = cteSum;

}

double PID::TotalError() {
}
