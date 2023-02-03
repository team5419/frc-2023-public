package frc.robot.commands;

import java.util.function.Supplier;

import org.opencv.core.RotatedRect;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.AprilTags;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.Util;

public class SpecialRamseteSwerve extends RamseteSwerve {
    enum State {
        PREDIAGONAL, 
        DIAGONAL,
        POSTDIAGONAL
    };
    private boolean useClosest; 
    private State state;
    private double targetX;
    private double targetY;
    private boolean imBasic;
    public SpecialRamseteSwerve(Swerve drivetrain, Vision vision, boolean useClosest, boolean imBasic) {
        super(drivetrain, vision, new Pose2d(), false);
        this.useClosest = useClosest;
        this.state = State.PREDIAGONAL;
        this.targetX = 0.0;
        
    }
    public void initialize() {
        System.out.println("Special init");
        int num = drivetrain.currentNum;
        int height = drivetrain.currentHeight;
        Pose2d pose = drivetrain.pose();
        Vision.Team team = vision.team;
        double currentY = pose.getY();
        if(team == Vision.Team.RED) {
            currentY = AprilTags.totalY - currentY;
        }
        if(useClosest) {
            int closestNum = 0;
            for(int i = 1; i < AprilTags.yPositions.length; i++) {
                if(AprilTags.yPositions[i] > currentY) {
                    closestNum = (Math.abs(AprilTags.yPositions[i - 1] - currentY) < Math.abs(AprilTags.yPositions[i] - currentY)) ? (i - 1) : i;
                    break;
                }
            }
            num = closestNum;
        }
        boolean isCone = (num - 1) % 3 != 0; 
        Rotation2d converted = Rotation2d.fromDegrees(isCone ? AprilTags.coneRotation : AprilTags.cubeRotation);
        double effectiveX = pose.getX();
        this.targetX = isCone ? AprilTags.coneDists[height] : AprilTags.cubeDists[height];
        this.targetY = AprilTags.yPositions[num];
        if(team == Vision.Team.RED) {
            this.targetY = AprilTags.totalY - targetY;
        }
        int section = Util.getSection(currentY);
        this.goal = pose;
        if(section == Util.getSection(AprilTags.yPositions[num]) && section != -1) {
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
