package frc.robot.commands;
import frc.robot.Util;

import java.lang.ModuleLayer.Controller;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GenericShootIntake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Swerve.AlignState;
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
    private boolean cones;
    private Lights lights;
    private int currentHeight;
    public SpecialRamseteTurn(Swerve drivetrain, Vision vision, XboxController controller, GenericShootIntake coner, GenericShootIntake cuber, Lights lights) {
        this.coner = coner;
        this.vision = vision;
        this.cuber = cuber;
        this.swerve = drivetrain;
        this.targetRotation = 0.0;
        this.isFinished = false;
        this.hasSeenTag = false;
        this.driver = controller;
        this.lights = lights;
        cones = false;
        currentHeight = 0;
        addRequirements(drivetrain, coner.subsystem(), cuber.subsystem());
    }
    public void initialize() {
        isFinished = false;
        hasSeenTag = false;
        System.out.println("Special init");
        cones = swerve.usingCones;
        currentHeight = swerve.currentHeight;
        GenericShootIntake shooter = cones ? coner : cuber;
        this.targetRotation = shooter.getAngle();
        lights.setColor(255, 0, 0);
        swerve.isAligning = AlignState.NOT;
    }

    public void execute() {
        GenericShootIntake shooter = cones ? coner : cuber;
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
            if(dtheta == 0 && (cones || hasSeenTag)) {
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
            GenericShootIntake shooter = cones ? coner : cuber;
            shooter.stop(TargetHeights.heights[currentHeight]);
            lights.off(swerve);
            return;
        }
        if(isFinished) {
            swerve.isAligning = AlignState.CONTROLLERON;
            CommandBase regularer = 
                cones ? new AutoAlign(swerve, coner, vision, driver, coner.getLimelightDistance(TargetHeights.heights[currentHeight]), currentHeight, lights)
                : new SpecialRamseteSwerve(swerve, vision, driver, cuber, true, currentHeight, false, new RamseteOptions(), lights);// if we're on cones, up epsilons hella and don't enforce a speed limit so we're fast before limelight
            regularer.andThen(new Shoot(coner, cuber, swerve, lights));
        }
    }
}
