package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.Util;
import frc.robot.Constants.SwerveDriveConstants;

public class SwerveDrive extends CommandBase {
    private XboxController driver;
    private XboxController codriver;
    private Swerve drivetrain;
    private boolean letGo;

    public SwerveDrive(Swerve drivetrain, XboxController _driver, XboxController _codriver){
        driver = _driver;
        codriver = _codriver;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        letGo = true;
    }

    public void initialize() {
        letGo = true;
    }
    public void execute() {
        int pov = codriver.getPOV();
        if(pov == -1) {
            letGo = true;
        } else if(letGo) {
            letGo = false;
            if((pov <= SwerveDriveConstants.dPadInputRange || pov >= 360 - SwerveDriveConstants.dPadInputRange) && drivetrain.currentHeight < 2) {
                drivetrain.currentHeight++;
            }
            if(pov >= 180 - SwerveDriveConstants.dPadInputRange && pov <= 180 + SwerveDriveConstants.dPadInputRange && drivetrain.currentHeight > 0) {
                drivetrain.currentHeight--;
            }
        } 
        drivetrain.drive(
            Util.deadband(-driver.getLeftY(), SwerveDriveConstants.controllerDeadband) * SwerveDriveConstants.speedMultiplier,
            Util.deadband(-driver.getLeftX(), SwerveDriveConstants.controllerDeadband) * SwerveDriveConstants.speedMultiplier, 
            Util.deadband(-driver.getRightX(), SwerveDriveConstants.controllerDeadband) * SwerveDriveConstants.turnMultiplier, true, true);
    }
    public boolean isFinished() {
        return false;
    }
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
