package frc.robot.commands.driving;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Cuber;
import frc.robot.subsystems.Swerve;
import frc.robot.Util;
import frc.robot.Constants.SwerveDriveConstants;


import frc.robot.subsystems.Vision;

public class SwerveDrive extends CommandBase {
    private XboxController driver;
    private XboxController codriver;
    private Swerve drivetrain;
    private boolean letGo;
    private Cuber cuber;
    private Vision vision;
    public SwerveDrive(Swerve drivetrain, XboxController _driver, XboxController _codriver, Cuber cuber, Vision _vision){
        driver = _driver;
        codriver = _codriver;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        letGo = true;
        this.cuber = cuber;
        this.vision = _vision;
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
            if((pov <= SwerveDriveConstants.dPadInputRange || pov >= 360 - SwerveDriveConstants.dPadInputRange) && drivetrain.currentHeight < 3) {
                drivetrain.currentHeight++;
                vision.setPipelineToHigh(true);
            }
            if(pov >= 90 - SwerveDriveConstants.dPadInputRange && pov <= 90 + SwerveDriveConstants.dPadInputRange && cuber.offset < 2) {
                cuber.offset++;
            }
            if(pov >= 270 - SwerveDriveConstants.dPadInputRange && pov <= 270 + SwerveDriveConstants.dPadInputRange && cuber.offset > 0) {
                cuber.offset--;
            }
            if(pov >= 180 - SwerveDriveConstants.dPadInputRange && pov <= 180 + SwerveDriveConstants.dPadInputRange && drivetrain.currentHeight > 0) {
                drivetrain.currentHeight--;
                vision.setPipelineToHigh(false);
            }
        } 
        drivetrain.drive(
            Util.deadband(-driver.getLeftY(), SwerveDriveConstants.controllerDeadband) * SwerveDriveConstants.speedMultiplier,
            Util.deadband(-driver.getLeftX(), SwerveDriveConstants.controllerDeadband) * SwerveDriveConstants.speedMultiplier, 
            Util.deadband(-driver.getRightX(), SwerveDriveConstants.controllerDeadband) * SwerveDriveConstants.turnMultiplier, true, false);
    }
    public boolean isFinished() {
        return false;
    }
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
