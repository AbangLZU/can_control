/**
 * @author [Adam Shan]
 * @email [shenzb12@lzu.edu.cn]
 * @create date 2018-06-20 11:05:46
 * @modify date 2018-06-20 11:05:46
 * @desc [pid controller header]
*/


#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <limits>
#include <algorithm>

#define MAX_THROTTLE_PERCENTAGE (0.8) //max throttle percentage

class PIDControl
{
  public:
    PIDControl();
    ~PIDControl();

    void initial(double kp, double ki, double kd);
    void resetError();
    double step(double cte, double sample_time);
    void updateError(double cte, double sample_time);
    void setRange(double min, double max);
    void setGains(double kp, double ki, double kd);

  private:
    /* data */
    double kp_;
    double ki_;
    double kd_;

    double p_error_, i_error_, d_error_;
    bool is_initialized_;

    double min_num_, max_num_;
};

#endif