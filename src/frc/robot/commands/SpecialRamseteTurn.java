package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GenericShootIntake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.Util;
import frc.robot.Constants.SwerveDriveConstants;

public class SpecialRamseteTurn extends CommandBase {
    private Swerve swerve;
    private double targetRotation;
    private GenericShootIntake coner;
    private GenericShootIntake cuber;
    private boolean isFinished;
    private Vision vision;
    private boolean hasSeenTag;
    private XboxController driver;
    public SpecialRamseteTurn(Swerve drivetrain, Vision vision, XboxController controller, GenericShootIntake coner, GenericShootIntake cuber) {
        this.coner = coner;
        this.vision = vision;
        this.cuber = cuber;
        this.swerve = drivetrain;
        this.targetRotation = 0.0;
        this.isFinished = false;
        this.hasSeenTag = false;
        this.driver = controller;
    }
    public void initialize() {
        isFinished = false;
        hasSeenTag = false;
        System.out.println("Special init");
        boolean isCone = swerve.currentNum != 1; 
        GenericShootIntake shooter = isCone ? coner : cuber;
        this.targetRotation = shooter.getAngle();
        
    }

    public void execute() {
        double theta = swerve.angle();
        double target = Math.round((theta - targetRotation) / 360.0) * 360.0 + targetRotation;

        double dtheta = 1 * SwerveDriveConstants.pTheta * (Math.PI / 180.0) * Util.deadband(target - theta, 10.0);
        //System.out.println(dtheta);
        //System.out.println("theta: ${DriveConstants.pTheta * (Math.PI / 180) * (target - theta)}");
        swerve.drive(
            Util.deadband(-driver.getLeftY(), SwerveDriveConstants.controllerDeadband) * SwerveDriveConstants.speedMultiplier,
            Util.deadband(-driver.getLeftX(), SwerveDriveConstants.controllerDeadband) * SwerveDriveConstants.speedMultiplier, 
            dtheta, true, true);
        if(vision.seesTag) {
            hasSeenTag = true;
        }
        isFinished = dtheta == 0 && swerve.getAverageSpeed() < 0.2 && hasSeenTag;
    }

    public boolean isFinished() {
        return isFinished;
    }

    public void end(boolean interrupted) {
        swerve.stop();
    }
}
