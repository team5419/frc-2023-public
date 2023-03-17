package frc.robot.classes;
public class PID {
    public double p;
    public double i;
    public double d;
    public double f;
    public PID(double _p, double _i, double _d) {
        p = _p;
        i = _i;
        d = _d;
        f = 0.0;
    }
    public PID(double _p, double _i, double _d, double _f) {
        p = _p;
        i = _i;
        d = _d;
        f = _f;
    }
}