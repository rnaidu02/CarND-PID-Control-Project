#include <vector>
#include <iostream>
#include <limits>
#include <cmath>
#include <numeric>
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

    errorTolerance = 0.1;       //From twiddle sample from the lesson
    dpList = {0.1, .1, .1};         // Set this as given in the sample from the lesson
    dBestError = std::numeric_limits<double>::max();    //Max number
    dTotalError = 0.0;
    nRunLimit  = 1;           //
    nRunRangeLimit = 1;      //set it to 1000 for twiddle to Calcuate params (Kp, Ki, Kd and dp errors)
    nCurrentStep = 0;


    isBestErrorInitalized = false;
    isFirstTimeAdded = isSecondTimeAdded = false;
    pramInProgress = 0;         //First param in progress
    tPhase = PHASE1;           //Initial phase

    std::cout << "p, i, d kps = " << this->Kp << ", " << this->Ki << ", " << this->Kd << std::endl;
}


void PID::UpdateError(double cte) {

    /*
    if (cteList.size() > 0){
        prev_cte = cteList.back();
    }
    */
    if (cteList.size() >= nCTEValues){
        //If the list grows more than max allowed, remove the first element
        //i.e keep the lat 20 from the list
        cteList.erase(cteList.begin());
        cteSum -= cteList.front();
    }
    cteList.push_back(cte);

    if (nCurrentStep == 0 ){
        prev_cte = cte;
    }
    cteSum += cte;
    p_error = cte;
    d_error = cte - prev_cte;
    i_error = cteSum;

    //Calculate the total error if the current step is more than <nRunLimit>
    if ((nCurrentStep % (nRunLimit+nRunRangeLimit))  >= nRunLimit){
        dTotalError += std::pow(cte, 2);
    }

    cout << "Mode val = " << (nCurrentStep % (nRunLimit+nRunRangeLimit)) << endl;
    //Only enter this loop if the dTotalError is calculated nRunLimit times
    if ((TotalError() > errorTolerance)&&((nCurrentStep % (nRunLimit+nRunRangeLimit))  == nRunLimit)){

        if (!isBestErrorInitalized){
            dBestError = dTotalError;
            isBestErrorInitalized = true;

        }
        cout << "Before twiddle code" << endl;
        switch (tPhase){
            case PHASE1:
                UpdateParamWithdp(pramInProgress, dpList[pramInProgress]);
                tPhase = PHASE2;
                break;
            case PHASE2:
                if (dTotalError < dBestError){
                    dBestError = dTotalError;
                    dpList[pramInProgress] *= 1.1;
                    tPhase = PHASE4;
                } else {
                    UpdateParamWithdp(pramInProgress, -2*dpList[pramInProgress]);
                    tPhase = PHASE3;
                }
                break;
            case PHASE3:
                if (dTotalError < dBestError){
                    dBestError = dTotalError;
                    dpList[pramInProgress] *= 1.1;
                    tPhase = PHASE4;
                }else {
                    UpdateParamWithdp(pramInProgress, dpList[pramInProgress]);
                    dpList[pramInProgress] *= 0.9;
                    tPhase = PHASE4;
                }
                break;
            case PHASE4:
                tPhase = PHASE1;
                pramInProgress = (pramInProgress+1)%3;

                cout << "Current Step at which the param updates are done" << nCurrentStep << endl;
                cout << "After param update dp coeff" << dpList[0] << ", " <<  dpList[1] << ", " << dpList[2] << endl;
                cout << "After param update K coeff" << Kp << ", " << Ki << ", " << Kd << endl;
                break;

        }

        //set the dTotalError to 0 to re-init for the error calculation
        dTotalError = 0.0;
        //reset the intgral error to 0
        cteSum = 0.0;
        cteList.clear();
    }

    nCurrentStep++;
}

double PID::TotalError() {
    //return std::accumulate(dpList.rbegin(), dpList.rend(), 0);
    double sum = fabs(dpList[0])+fabs(dpList[1])+fabs(dpList[2]);
    return sum;
}

/*
 * UpdateParamWithdp(unsigned int paramIndex, double dpVL)
*/
void PID::UpdateParamWithdp(unsigned int paramIndex, double dpVal){
    cout << "UpdateParamWithdp Before K coeff" << this->Kp << ", " << this->Ki << ", " << this->Kd << endl;
    switch(paramIndex){
        case 0:
            this->Kp += dpVal;
            break;
        case 1:
            this->Ki += dpVal;
            break;
        case 2:
            this->Kd += dpVal;
            break;
    }
    cout << "UpdateParamWithdp After K coeff" << this->Kp << ", " << this->Ki << ", " << this->Kd << endl;
}
