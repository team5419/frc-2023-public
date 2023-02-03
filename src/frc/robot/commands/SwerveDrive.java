package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.Util;
import frc.robot.Constants.Drive;

public class SwerveDrive extends CommandBase {
    XboxController driver;
    Swerve drivetrain;

    public SwerveDrive(Swerve drivetrain, XboxController _driver){
        driver = _driver;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    public void initialize() {

    }
    public void execute() {
        int pov = driver.getPOV();
        if(pov != -1) {
            if((pov <= Drive.dPadInputRange || pov >= 360 - Drive.dPadInputRange) && drivetrain.currentHeight < 2) {
                drivetrain.currentHeight++;
            }
            if(pov >= 90 - Drive.dPadInputRange && pov <= 90 + Drive.dPadInputRange && drivetrain.currentNum < 2) {
                drivetrain.currentNum++;
            }
            if(pov >= 180 - Drive.dPadInputRange && pov <= 180 + Drive.dPadInputRange && drivetrain.currentHeight > 0) {
                drivetrain.currentHeight--;
            }
            if(pov >= 270 - Drive.dPadInputRange && pov <= 270 + Drive.dPadInputRange && drivetrain.currentNum > 0) {
                drivetrain.currentNum--;
            }
        }
        drivetrain.drive(
            Util.deadband(-driver.getLeftY(), Drive.controllerDeadband) * Drive.speedMultiplier,
            Util.deadband(-driver.getLeftX(), Drive.controllerDeadband) * Drive.speedMultiplier, 
            Util.deadband(-driver.getRightX(), Drive.controllerDeadband) * Drive.turnMultiplier, true, false);
    }
    public boolean isFinished() {
        return false;
    }
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
