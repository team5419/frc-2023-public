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
import frc.robot.classes.RamseteOptions;

public class SpecialRamseteTurn extends CommandBase {
    private Swerve swerve;
    private double targetRotation;
    private GenericShootIntake coner;
    private GenericShootIntake cuber;
    private boolean isFinished;
    private Vision vision;
    private boolean hasSeenTag;
    private XboxController driver;
    private int currentNum;
    private int currentHeight;
    public SpecialRamseteTurn(Swerve drivetrain, Vision vision, XboxController controller, GenericShootIntake coner, GenericShootIntake cuber) {
        this.coner = coner;
        this.vision = vision;
        this.cuber = cuber;
        this.swerve = drivetrain;
        this.targetRotation = 0.0;
        this.isFinished = false;
        this.hasSeenTag = false;
        this.driver = controller;
        currentNum = 0;
        currentHeight = 0;
        addRequirements(drivetrain, coner.subsystem(), cuber.subsystem());
    }
    public void initialize() {
        isFinished = false;
        hasSeenTag = false;
        System.out.println("Special init");
        currentNum = swerve.currentNum;
        currentHeight = swerve.currentHeight;
        boolean isCone = currentNum != 1; 
        GenericShootIntake shooter = isCone ? coner : cuber;
        this.targetRotation = shooter.getAngle();
    }

    public void execute() {
        GenericShootIntake shooter = (currentNum != 1) ? coner : cuber;
        if(shooter.prepsByDefault()) {
            shooter.setup(TargetHeights.heights[currentHeight]);
        }

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
        //if(swerve.getAverageSpeed() < 0.2) { may not need speed requirement for this?
            if(dtheta == 0 && hasSeenTag) {
                isFinished = true;
            }
        //}
    }

    public boolean isFinished() {
        return isFinished;
    }

    public void end(boolean interrupted) {
        swerve.stop();
        if(interrupted) {
            GenericShootIntake shooter = (currentNum != 1) ? coner : cuber;
            shooter.stop(TargetHeights.heights[currentHeight]);
            return;
        }
        if(isFinished) {
            SpecialRamseteSwerve regularer = new SpecialRamseteSwerve(swerve, vision, driver, currentNum == 1 ? cuber : coner, true, currentNum, currentHeight, 
                currentNum == 1 ? new RamseteOptions() : new RamseteOptions(false, 5.0)); // if we're on cones, up epsilons hella and don't enforce a speed limit so we're fast before limelight
            regularer.schedule();
        }
    }
}
