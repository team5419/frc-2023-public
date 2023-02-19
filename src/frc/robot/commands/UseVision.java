package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class UseVision extends CommandBase {
    private Swerve drivetrain;
    private boolean setpoint;

    public UseVision(Swerve drivetrain, boolean setpoint){
        this.drivetrain = drivetrain;
        this.setpoint = setpoint;
    }

    public void initialize() {
        drivetrain.usingVision = setpoint;
    }
    public void execute() {
    }
    public boolean isFinished() {
        return true;
    }
    public void end(boolean interrupted) {
    }
}
