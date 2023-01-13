package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Pose2d;

public class goToPosition extends CommandBase {
    XboxController joystick = new XboxController(1);
    Pose2d desiredPosition;
    
    public void initialize() {
        
    }
    public void execute() {
        
    }
    public boolean isFinished() {
        return Swerve.pose == 
    }
    public void end(boolean interrupted) {
        
    }
}
