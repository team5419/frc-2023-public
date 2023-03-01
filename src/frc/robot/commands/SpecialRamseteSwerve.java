package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.GenericShootIntake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.Util;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.TargetHeights;
import frc.robot.classes.RamseteOptions;

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
    private GenericShootIntake shooter;
    private boolean cones;
    private int height;
    private Lights lights;
    public SpecialRamseteSwerve(Swerve drivetrain, Vision vision, GenericShootIntake shooter, boolean imBasic, int height, boolean cones, RamseteOptions options, Lights lights) {
        super(drivetrain, vision, new Pose2d(), options);
        addRequirements(shooter.subsystem());
        this.state = State.PREDIAGONAL;
        this.targetX = 0.0;
        this.targetY = 0.0;
        this.imBasic = imBasic;
        this.shooter = shooter;
        this.cones = cones;
        this.height = height;
        this.lights = lights;
    }
    public void initialize() {
        System.out.println("Special init");
        Pose2d pose = drivetrain.pose();
        Alliance team = vision.team();
        double currentY = pose.getY();
        if(team == Alliance.Red) {
            currentY = AprilTagConstants.totalY - currentY;
        }
            int closestNum = 0;
            double closestDist = -1;
            
            for(int i = 0; i < AprilTagConstants.yPositions.length; i++) {
                if(cones == (i % 3 == 1)) {
                    continue;
                }
                double dist = Math.abs(AprilTagConstants.yPositions[i] - currentY);
                if(closestDist == -1 || dist < closestDist) {
                    closestDist = dist;
                    closestNum = i;
                }
            }
        Rotation2d converted = Rotation2d.fromDegrees(shooter.getAngle()/* if we have front and back cameras, use this, otherwise --> */ /*AprilTagConstants.cameraAngle*/);
        double effectiveX = pose.getX();
        this.targetX = shooter.getDistance(TargetHeights.heights[height]);
        this.targetY = AprilTagConstants.yPositions[closestNum] + shooter.getOffset() + AprilTagConstants.targetYOffset;
        if(team == Alliance.Red) {
            this.targetY = AprilTagConstants.totalY - targetY;
        }
        int section = Util.getSection(currentY);
        this.goal = new Pose2d(targetX, targetY, converted);
        System.out.println("going to x: " + targetX + ", y: " + targetY + ", theta: " + converted.getDegrees());
        if((section == Util.getSection(AprilTagConstants.yPositions[closestNum]) && section != -1) || imBasic) {
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
        if(shooter.prepsByDefault()) {
            shooter.setup(TargetHeights.heights[height]);
        }
        super.execute();
    }

    public boolean isFinished() {
        if(isFinished) {
            lights.setColor(0, 255, 0);
            isFinished = false;
            if(state == State.PREDIAGONAL) {
                state = State.DIAGONAL;
                this.goal = new Pose2d(Math.min(Math.max(AprilTagConstants.xPosBeforeBarriers, targetX), AprilTagConstants.xEndOfChargingStation), targetY, this.goal.getRotation());
            } else if(state == State.DIAGONAL) {
                state = State.POSTDIAGONAL;
                this.goal = new Pose2d(targetX, targetY, this.goal.getRotation());
            } else {
                return true;//cones;
            }
        } else {
            lights.setColor(255, 0, 0);
        }
        return false;
    }

    public void end(boolean interrupted) {
        System.out.println("SPECIAL SWERVE ENDED!!");
        super.end(interrupted);
    }
}
