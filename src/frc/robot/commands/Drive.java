package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase {
    XboxController joystick = new XboxController(1);
    Drivetrain drivetrain = new Drivetrain();
    public void initialize() {
        
    }
    public void execute() {
        drivetrain.drive(joystick.getLeftY(),joystick.getRightX());
    }
    public boolean isFinished() {
        return false;
    }
    public void end(boolean interrupted) {
        
    }
}
