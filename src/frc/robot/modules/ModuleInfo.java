package frc.robot.modules;

class ModuleInfo {
    public int driverPort;
    public int turnerPort;
    public boolean driveInverted;
    public boolean turnInverted;
    public int cancoderPort;
    public double offset;
    public ModuleInfo(int _driverPort, int _turnerPort, boolean _driveInverted, boolean _turnInverted, int _cancoderPort, double _offset) {
        this.driverPort = _driverPort;
        this.turnerPort = _turnerPort;
        this.driveInverted = _driveInverted;
        this.turnInverted = _turnInverted;
        this.cancoderPort = _cancoderPort;
        this.offset = _offset;
    }
}