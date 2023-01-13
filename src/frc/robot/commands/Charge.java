package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.button;

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.XboxController;

public class Charge extends CommandBase {
    
   
    public ProtoRoutine(Drivetrain drivetrain) {
        addCommands(
            new RamseteAction(drivetrain, new Pose2d[] { 
                new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
                new Pose2d(1.0, 0.0, new Rotation2d(0.0))
   
    public void initialize() {

    }
    public void execute() { 

    }
    public boolean isFinished() {
        return true;
    }
    public void end(boolean interrupted) {
        
    }
}
