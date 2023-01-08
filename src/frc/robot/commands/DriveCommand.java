package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.XboxController;

public class DriveCommand extends CommandBase {

    XboxController driver;
    DriveTrain drivetrain = new DriveTrain();

    public DriveCommand(XboxController _driver){
        driver = _driver;
    }

    public void initialize() {

    }
    public void execute() {
        drivetrain.drive(driver.getLeftY(), driver.getRightX());
    }
    public boolean isFinished() {
        return true;
    }
    public void end(boolean interrupted) {
        
    }
}
