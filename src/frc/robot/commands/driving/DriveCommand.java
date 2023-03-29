package frc.robot.commands.driving;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveCommand extends CommandBase {
    XboxController driver;
    Drivetrain drivetrain;
    private boolean slow;

    public DriveCommand(Drivetrain drivetrain, XboxController _driver, boolean slow){
        driver = _driver;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        this.slow = slow;
    }

    public void initialize() {

    }
    public void execute() {
        drivetrain.drive(driver.getLeftY(), driver.getRightX(), slow);
    }
    public boolean isFinished() {
        return false;
    }
    public void end(boolean interrupted) {
        drivetrain.drive(0.0, 0.0, false);
    }
}
