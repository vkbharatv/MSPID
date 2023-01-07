class PID {
private:
    double lastError, LastValue, rateError, w, out_c;
public:
    double T_e, error, out_n, T_c, T_n, T_p,cumError;
    double kp, ki, kd;
    double MaxU, MinU;
    PID(double Kc, double TauI, double TauD, double maxU, double minU);
    double PIDval(double R, double S);
};
