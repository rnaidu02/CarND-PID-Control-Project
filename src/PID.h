#ifndef PID_H
#define PID_H

enum TWIDDLE_PHASES {PHASE1, PHASE2, PHASE3, PHASE4};
class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  /*
  * Store prev_cte, int_cte
  */
  double prev_cte;
  double int_cte;

  /*
   * If it is the first msmt?
  */
  bool isFirstMsmt;

  /*
   * Number of samples to keep for cte
  */
  unsigned int nCTEValues;

  std::vector<double > cteList;
  double cteSum;

  /*
   * Twiddle related variables
  */

  double errorTolerance;        //tolerance for total error
  std::vector<double > dpList;  //store dp error for PID
  double dBestError;            //store best error
  double dTotalError;           //store total error measured
  unsigned int nRunLimit;       //offset number of iterations for run function to measure total error
  unsigned int nRunRangeLimit;  //total number of iterations for twiddle function to run
  unsigned int nCurrentStep;    //Current step counter

  bool  isBestErrorInitalized; //Is it first time in while Loop?
  unsigned int pramInProgress;  //param update in progress (possible values 0, 1, 2 for Kp, Ki, Kd)

  bool isFirstTimeAdded;        //First time dp added to param
  bool isSecondTimeAdded;        //Second time dp added to param

  TWIDDLE_PHASES    tPhase;    //Twiddle algo phases
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
   * UpdateParamWithdp(unsigned int paramIndex, double dpVL)
  */
  void UpdateParamWithdp(unsigned int paramIndex, double dpVal);
};

#endif /* PID_H */
