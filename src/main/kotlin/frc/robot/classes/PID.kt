package frc.robot.classes;
open class PID {
    val p: Double;
    val i: Double;
    val d: Double;
    constructor(_p: Double, _i: Double, _d: Double) {
        this.p = _p;
        this.i = _i;
        this.d = _d;
    }
}