package frc.robot.commands;
// import frc.robot.Constants.Drive;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ResetGyro extends CommandBase {
    private Swerve swerve;
    private double angle;

    public ResetGyro(Swerve _swerve) {
        swerve = _swerve;
    }

    public ResetGyro(Swerve _swerve, double _angle) {
        swerve = _swerve;
        angle = _angle;
    }

    public void initialize() {}

    public void execute() {}

    public void end(boolean interrupted) {
        swerve.gyro.setYaw(angle);
    }

    public boolean isFinished() {
        return true;
    }
}