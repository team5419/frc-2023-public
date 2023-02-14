package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GenericShootIntake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.Util;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.TargetHeights;

public class SpecialRamseteTurn extends CommandBase {
    private Swerve swerve;
    private double targetRotation;
    private GenericShootIntake coner;
    private GenericShootIntake cuber;
    private FinishState isFinished;
    private Vision vision;
    private boolean hasSeenTag;
    private XboxController driver;
    private int currentNum;
    private int currentHeight;
    private double overrideAngleOffset;
    private enum FinishState {
        NOT, 
        REGULAR,
        SKIP
    }
    public SpecialRamseteTurn(Swerve drivetrain, Vision vision, XboxController controller, GenericShootIntake coner, GenericShootIntake cuber) {
        this.coner = coner;
        this.vision = vision;
        this.cuber = cuber;
        this.swerve = drivetrain;
        this.targetRotation = 0.0;
        this.isFinished = FinishState.NOT;
        this.hasSeenTag = false;
        this.driver = controller;
        currentNum = 0;
        currentHeight = 0;
        overrideAngleOffset = 0.0;
    }
    public SpecialRamseteTurn(Swerve drivetrain, Vision vision, XboxController controller, GenericShootIntake coner, GenericShootIntake cuber, double overrideAngle) {
        this.coner = coner;
        this.vision = vision;
        this.cuber = cuber;
        this.swerve = drivetrain;
        this.targetRotation = 0.0;
        this.isFinished = FinishState.NOT;
        this.hasSeenTag = false;
        this.driver = controller;
        currentNum = 0;
        currentHeight = 0;
        overrideAngleOffset = overrideAngle;
    }
    public void initialize() {
        vision.on();
        isFinished = FinishState.NOT;
        hasSeenTag = false;
        System.out.println("Special init");
        currentNum = swerve.currentNum;
        currentHeight = swerve.currentHeight;
        boolean isCone = currentNum != 1; 
        GenericShootIntake shooter = isCone ? coner : cuber;
        this.targetRotation = shooter.getAngle() + overrideAngleOffset;
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
        if(swerve.getAverageSpeed() < 0.2) {
            // if(currentNum != 1) {
            //     if(vision.isTargetFound()) {
            //         isFinished = FinishState.SKIP;
            //     }
            /* } else*/ if(dtheta == 0 && hasSeenTag) {
                isFinished = FinishState.REGULAR;
            }
        }
    }

    public boolean isFinished() {
        return isFinished != FinishState.NOT;
    }

    public void end(boolean interrupted) {
        System.out.println("SPECIAL TURN ENDED!!!");
        swerve.stop();
        if(interrupted) {
            System.out.println("I was interrupted");
            return;
        }
        if(isFinished == FinishState.SKIP) {
            AutoAlign aligner = new AutoAlign(swerve, coner, vision, driver, coner.getLimelightDistance(TargetHeights.heights[currentHeight]));
            aligner.schedule();
        } else if(isFinished == FinishState.REGULAR) {
            SpecialRamseteSwerve regularer = new SpecialRamseteSwerve(swerve, vision, driver, coner, cuber, true);
            regularer.schedule();
        }
    }
}
