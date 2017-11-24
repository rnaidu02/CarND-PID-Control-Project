#ifndef PID_H
#define PID_H

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
};

#endif /* PID_H */
