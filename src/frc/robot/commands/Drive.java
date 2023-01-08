package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase {
    XboxController xbox = new XboxController(0);
    public void initialize() {

    }
    public void execute() {
        Drivetrain.drive(xbox.getLeftY(),xbox.getRightX());
    }
    public boolean isFinished() {
        return true;
    }
    public void end(boolean interrupted) {
        
    }
}
