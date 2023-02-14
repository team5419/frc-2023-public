package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.GenericShootIntake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.Util;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.TargetHeights;

public class SpecialRamseteSwerve extends RamseteSwerve {
    enum State {
        PREDIAGONAL, 
        DIAGONAL,
        POSTDIAGONAL
    };
    enum ControllerState {
        NOTOFFYET,
        OFF,
        ONAGAIN
    }
    private State state;
    private double targetX;
    private double targetY;
    private boolean imBasic;
    private XboxController driver;
    private GenericShootIntake coner;
    private GenericShootIntake cuber;
    private ControllerState controller;
    private boolean isCone;
    private int height;
    private static int count;
    public SpecialRamseteSwerve(Swerve drivetrain, Vision vision, XboxController driver, GenericShootIntake coner, GenericShootIntake cuber, boolean imBasic) {
        super(drivetrain, vision, new Pose2d(), false, false);
        this.state = State.PREDIAGONAL;
        this.targetX = 0.0;
        this.targetY = 0.0;
        this.imBasic = imBasic;
        this.driver = driver;
        this.coner = coner;
        this.cuber = cuber;
        this.controller = ControllerState.NOTOFFYET;
        this.isCone = false;
        this.height = 0;
        ShuffleboardTab tab = Shuffleboard.getTab("Master");
        // count++;
        // tab.addNumber("ramsete target y + " + count, () -> goal.getY());
    }
    public void initialize() {
        this.controller = ControllerState.NOTOFFYET;
        System.out.println("Special init");
        int num = drivetrain.currentNum;
        isCone = num != 1; 
        height = drivetrain.currentHeight;
        Pose2d pose = drivetrain.pose();
        Alliance team = vision.team();
        double currentY = pose.getY();
        if(team == Alliance.Red) {
            currentY = AprilTagConstants.totalY - currentY;
        }
            int closestNum = AprilTagConstants.yPositions.length - 1;
            for(int i = 1; i < AprilTagConstants.yPositions.length; i++) {
                if(AprilTagConstants.yPositions[i] > currentY) {
                    closestNum = (Math.abs(AprilTagConstants.yPositions[i - 1] - currentY) < Math.abs(AprilTagConstants.yPositions[i] - currentY)) ? (i - 1) : i;
                    break;
                }
            }
            num += 3 * (closestNum / 3); // sketchy fr
            System.out.println(num);
        GenericShootIntake shooter = isCone ? coner : cuber;
        Rotation2d converted = Rotation2d.fromDegrees(shooter.getAngle()/* if we have front and back cameras, use this, otherwise --> */ /*AprilTagConstants.cameraAngle*/);
        double effectiveX = pose.getX();
        this.targetX = shooter.getDistance(TargetHeights.heights[height]);//isCone ? AprilTagConstants.coneDists[height] : AprilTagConstants.cubeDists[height];
        this.targetY = AprilTagConstants.yPositions[num] + shooter.getOffset() + AprilTagConstants.targetYOffset;
        if(team == Alliance.Red) {
            this.targetY = AprilTagConstants.totalY - targetY;
        }
        int section = Util.getSection(currentY);
        this.goal = new Pose2d(targetX, targetY, converted);
        System.out.println("going to x: " + targetX + ", y: " + targetY + ", theta: " + converted.getDegrees());
        if((section == Util.getSection(AprilTagConstants.yPositions[num]) && section != -1) || imBasic) {
            this.state = State.POSTDIAGONAL;
        } else {
            this.state = State.PREDIAGONAL;
            if(effectiveX > AprilTagConstants.xEndOfChargingStation) { // if hasn't made it to end of charging station, drive straight to end of station to avoid collision
                this.goal = new Pose2d(AprilTagConstants.xEndOfChargingStation, pose.getY(), converted);
            } else if(effectiveX < AprilTagConstants.xPosBeforeBarriers) {
                this.goal = new Pose2d(AprilTagConstants.xPosBeforeBarriers, pose.getY(), converted);
            }
        }
    }

    public void execute() {
        super.execute();
        if(Math.abs(driver.getLeftX()) > SwerveDriveConstants.controllerDeadband || Math.abs(driver.getLeftY()) > SwerveDriveConstants.controllerDeadband || Math.abs(driver.getRightX()) > SwerveDriveConstants.controllerDeadband || Math.abs(driver.getRightY()) > SwerveDriveConstants.controllerDeadband) {
            if(controller == ControllerState.OFF) {
                controller = ControllerState.ONAGAIN;
            }
        } else {
            controller = ControllerState.OFF;
        }
    }

    public boolean isFinished() {
        if(controller == ControllerState.ONAGAIN) {
            return true;
        }
        if(isFinished) {
            isFinished = false;
            if(state == State.PREDIAGONAL) {
                state = State.DIAGONAL;
                this.goal = new Pose2d(Math.min(Math.max(AprilTagConstants.xPosBeforeBarriers, targetX), AprilTagConstants.xEndOfChargingStation), targetY, this.goal.getRotation());
            } else if(state == State.DIAGONAL) {
                state = State.POSTDIAGONAL;
                this.goal = new Pose2d(targetX, targetY, this.goal.getRotation());
            } else {
                return isCone;
            }
        }
        return false;
    }

    public void end(boolean interrupted) {
        System.out.println("SPECIAL SWERVE ENDED!!");
        super.end(interrupted);
        if(isCone && controller != ControllerState.ONAGAIN) {
            AutoAlign aligner = new AutoAlign(drivetrain, coner, vision, driver, coner.getLimelightDistance(TargetHeights.heights[height]));
            aligner.schedule();
        }
    }
}
