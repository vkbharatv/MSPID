#include "PID.h"
#include <Arduino.h>
PID::PID(double Kc, double TauI, double TauD, double maxU, double minU)
{
    kp = Kc;
    ki = TauI;
    kd = TauD;
    MaxU = maxU;
    MinU = minU;
}

double PID::PIDval(double R, double S)
{
    T_e = (micros() - T_p) / 1000000;
    error = R - S;

    // cumError += (error *ki + w/ki) ;
    rateError = (error - lastError) / (T_e);
    out_n = kp * (error + cumError * (1 / ki) + kd * rateError);

    if (out_n > MaxU)
    {
        out_c = MaxU;
    }
    else if (out_n < MinU)
    {
        out_c = -MinU;
    }
    else
    {
        out_c = out_n;
    }

    // Anti-Windup Logic
    if (cumError > MaxU && out_n > MaxU)
        cumError = MaxU;
    else if (cumError < MinU && out_n < MinU)
        cumError = MinU;
    else
        cumError += (error)*T_e;
    //

    T_p = micros();
    lastError = error;
    return out_c;
}
