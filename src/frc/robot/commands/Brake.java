package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
public class Brake extends CommandBase {
    private Swerve drivetrain;
    public Brake(Swerve drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }
    public void initialize() {
        
    }
    public void execute() { 
        drivetrain.brake();
    }
    public boolean isFinished() {
        return false;
    }
    public void end(boolean interrupted) {
        
    }
}