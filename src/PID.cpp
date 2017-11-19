#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    Kp = Kp;
    Ki = Ki;
    Kd = Kd;
}

void PID::UpdateError(double cte) {
    
}

double PID::TotalError() {
}
