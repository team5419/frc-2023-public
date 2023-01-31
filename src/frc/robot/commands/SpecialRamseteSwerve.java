package frc.robot.commands;

import java.util.function.Supplier;

import org.opencv.core.RotatedRect;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.AprilTags;
import frc.robot.subsystems.Swerve;

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
    public SpecialRamseteSwerve(Swerve drivetrain, boolean useClosest) {
        super(drivetrain, new Pose2d());
        this.useClosest = useClosest;
        this.state = State.PREDIAGONAL;
        this.targetX = 0.0;
    }
    public void initialize() {
        System.out.println("Special init");
        int num = drivetrain.currentNum;
        int height = drivetrain.currentHeight;
        Pose2d pose = drivetrain.pose();
        double angle = drivetrain.angle();
        if(useClosest) {
            double currentY = pose.getY();
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
        double targetRotation = Math.round((angle  - (isCone ? AprilTags.coneRotation : AprilTags.cubeRotation)) / 360.0);
        targetRotation *= 360.0;
        targetRotation += isCone ? AprilTags.coneRotation : AprilTags.cubeRotation;
        Rotation2d converted = Rotation2d.fromDegrees(targetRotation);
        double effectiveX = pose.getX();
        double targetX = isCone ? AprilTags.coneDists[height] : AprilTags.cubeDists[height];
        double targetY = AprilTags.yPositions[num];
        
        this.state = State.PREDIAGONAL;
        this.targetX = targetX;
        System.out.println("targetX = " + targetX);
        System.out.println("targetY = " + targetY);
        System.out.println("target rotation = " + targetRotation);
        this.targetY = targetY;
        if(effectiveX > AprilTags.xEndOfChargingStation) { // if hasn't made it to end of charging station, drive straight to end of station to avoid collision
            this.goal = new Pose2d(AprilTags.xEndOfChargingStation, pose.getY(), converted);
        } else if(effectiveX < AprilTags.xPosBeforeBarriers) {
            this.goal = new Pose2d(AprilTags.xPosBeforeBarriers, pose.getY(), converted);
        } else {
            this.goal = pose;
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
