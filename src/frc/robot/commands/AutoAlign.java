package frc.robot.commands; // auto align using only limelight 
import frc.robot.Util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.TargetHeights;
import frc.robot.subsystems.Coner;
import frc.robot.subsystems.GenericShootIntake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
public class AutoAlign extends CommandBase {
    private Swerve drivetrain;
    private Vision vision;
    private double distance;
    private XboxController driver;
    private GenericShootIntake shooter;
    private int height;
    public AutoAlign(Swerve drivetrain, GenericShootIntake shooter, Vision vision, XboxController driver, double distance, int height) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.distance = distance;
        this.driver = driver;
        this.shooter = shooter;
        this.height = height;
        addRequirements(drivetrain, shooter.subsystem());
    }
    public void initialize() {
        vision.on();
    }
    public void execute() {
        double theta = drivetrain.angle();
        double target = Math.round((theta - LimelightConstants.desiredAngle) / 360.0) * 360.0 + LimelightConstants.desiredAngle;
        double turnDiff = Util.deadband(target - drivetrain.angle(), LimelightConstants.epsilonTurn); //calculates how many degrees to turn
        double turn = LimelightConstants.turnPID.calculate(turnDiff); //calculates amt to turn
        double leftDiff = Util.deadband(-vision.getHorizontalOffset(), LimelightConstants.epsilonHorizontal); //how far sidewyas to move across fied
        double left = LimelightConstants.horizontalPID.calculate(leftDiff);// ''
        double forwardDiff = Util.deadband( distance - LimelightConstants.desiredDistance - vision.getHorizontalDistance(), LimelightConstants.epsilonForward); //how far forward to go
        
        double forward = LimelightConstants.forwardPID.calculate(forwardDiff);
        if(!vision.isTargetFound()) {
            leftDiff = 0.0;
            forwardDiff = 0.0;
            left = 0.0;
            forward = 0.0;
        } else if(Math.abs(forwardDiff) < 0.2) {
            shooter.setup(TargetHeights.heights[height]);
        }
        drivetrain.drive(forward, -left, -turn, false, true);
    }
    public boolean isFinished() {
        return Math.abs(driver.getLeftX()) > SwerveDriveConstants.controllerDeadband || Math.abs(driver.getLeftY()) > SwerveDriveConstants.controllerDeadband || Math.abs(driver.getRightX()) > SwerveDriveConstants.controllerDeadband || Math.abs(driver.getRightY()) > SwerveDriveConstants.controllerDeadband;
    }
    public void end(boolean interrupted) {
        drivetrain.stop();
        vision.off();
    }
}