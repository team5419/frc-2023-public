package frc.robot.commands;

import java.util.function.Supplier;

import org.opencv.core.RotatedRect;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AprilTags;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.Util;

public class SpecialRamseteSwerve extends RamseteSwerve {
    enum State {
        PREDIAGONAL, 
        DIAGONAL,
        POSTDIAGONAL
    };
    private State state;
    private double targetX;
    private double targetY;
    private boolean imBasic;
    private XboxController driver;
    public SpecialRamseteSwerve(Swerve drivetrain, Vision vision, XboxController driver, boolean imBasic) {
        super(drivetrain, vision, new Pose2d(), false);
        this.state = State.PREDIAGONAL;
        this.targetX = 0.0;
        this.targetY = 0.0;
        this.imBasic = imBasic;
        this.driver = driver;
    }
    public void initialize() {
        System.out.println("Special init");
        int num = drivetrain.currentNum;
        boolean isCone = (num - 1) % 3 != 0; 
        int height = drivetrain.currentHeight;
        Pose2d pose = drivetrain.pose();
        Vision.Team team = vision.team;
        double currentY = pose.getY();
        if(team == Vision.Team.RED) {
            currentY = AprilTags.totalY - currentY;
        }
            int closestNum = 0;
            for(int i = 1; i < AprilTags.yPositions.length; i++) {
                if(AprilTags.yPositions[i] > currentY) {
                    closestNum = (Math.abs(AprilTags.yPositions[i - 1] - currentY) < Math.abs(AprilTags.yPositions[i] - currentY)) ? (i - 1) : i;
                    break;
                }
            }
            num += 3 * (closestNum / 3); // sketchy fr
        Rotation2d converted = Rotation2d.fromDegrees(isCone ? AprilTags.coneRotation : AprilTags.cubeRotation);
        double effectiveX = pose.getX();
        this.targetX = isCone ? AprilTags.coneDists[height] : AprilTags.cubeDists[height];
        this.targetY = AprilTags.yPositions[num];
        if(team == Vision.Team.RED) {
            this.targetY = AprilTags.totalY - targetY;
        }
        int section = Util.getSection(currentY);
        this.goal = pose;
        if((section == Util.getSection(AprilTags.yPositions[num]) && section != -1) || imBasic) {
            this.state = State.POSTDIAGONAL;
        } else {
            this.state = State.PREDIAGONAL;
            if(effectiveX > AprilTags.xEndOfChargingStation) { // if hasn't made it to end of charging station, drive straight to end of station to avoid collision
                this.goal = new Pose2d(AprilTags.xEndOfChargingStation, pose.getY(), converted);
            } else if(effectiveX < AprilTags.xPosBeforeBarriers) {
                this.goal = new Pose2d(AprilTags.xPosBeforeBarriers, pose.getY(), converted);
            }
        }
        
    }

    public boolean isFinished() {
        if(Math.abs(driver.getLeftX()) > Drive.controllerDeadband || Math.abs(driver.getLeftY()) > Drive.controllerDeadband || Math.abs(driver.getRightX()) > Drive.controllerDeadband || Math.abs(driver.getRightY()) > Drive.controllerDeadband) {
            return true;
        }
        if(isFinished) {
            isFinished = false;
            if(state == State.PREDIAGONAL) {
                state = State.DIAGONAL;
                this.goal = new Pose2d(Math.min(Math.max(AprilTags.xPosBeforeBarriers, targetX), AprilTags.xEndOfChargingStation), targetY, this.goal.getRotation());
            } else if(state == State.DIAGONAL) {
                state = State.POSTDIAGONAL;
                this.goal = new Pose2d(targetX, targetY, this.goal.getRotation());
            } else {
                System.out.println("finished");
                return true;
            }
        }
        return false;
    }
}
