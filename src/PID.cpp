#include <vector>
#include <iostream>
#include <limits>
#include <cmath>
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

    //nCTEValues = 200;   //remove this
    cteSum = 0.0;
    prev_cte = 0.0;

    errorTolerance = 0.2;       //From twiddle sample from the lesson
    dpList = {1, 1, 1};         // Set this as given in the sample from the lesson
    dBestError = std::numeric_limits<double>::max();    //Max number
    dTotalError = 0.0;
    nRunLimit  = 100;           //
    nRunRangeLimit = 1000;      //set it to 1000 for twiddle to Calcuate params (Kp, Ki, Kd and dp errors)
    nCurrentStep = 0;

    std::cout << "p, i, d kps = " << this->Kp << ", " << this->Ki << ", " << this->Kd << std::endl;
}


void PID::UpdateError(double cte) {

    /*
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
    */
    if (nCurrentStep == 0 ){
        prev_cte = cte;
    }
    cteSum += cte;
    p_error = cte;
    d_error = cte - prev_cte;
    i_error = cteSum;

    //Calculate the total error if the current step is more than <nRunLimit>
    if (nCurrentStep >= nRunLimit){
        dTotalError += std::pow(cte, 2);
    }



}

double PID::TotalError() {
}
