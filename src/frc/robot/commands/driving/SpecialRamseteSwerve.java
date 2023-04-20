package frc.robot.commands.driving;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Cuber;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.TargetHeights;
import frc.robot.classes.RamseteOptions;

public class SpecialRamseteSwerve extends RamseteSwerve {
    private double targetX;
    private double targetY;
    private Cuber shooter;
    private int height;
    private Lights lights;
    public SpecialRamseteSwerve(Swerve drivetrain, Vision vision, Cuber shooter, int height, RamseteOptions options, Lights lights) {
        super(drivetrain, vision, new Pose2d(), options);
        addRequirements(shooter.subsystem());
        this.targetX = 0.0;
        this.targetY = 0.0;
        this.shooter = shooter;
        this.height = height;
        this.lights = lights;
    }
    public void initialize() {
        Pose2d pose = drivetrain.pose();
        Alliance team = vision.team();
        double currentY = pose.getY();
        if(team == Alliance.Red) {
            currentY = AprilTagConstants.totalY - currentY;
        }
            int closestNum = 0;
            double closestDist = -1;
            
            for(int i = 1; i < AprilTagConstants.yPositions.length; i += 3) {
                double dist = Math.abs(AprilTagConstants.yPositions[i] - currentY);
                if(closestDist == -1 || dist < closestDist) {
                    closestDist = dist;
                    closestNum = i;
                }
            }
        Rotation2d converted = Rotation2d.fromDegrees(0.0);
        this.targetX = shooter.getDistance(TargetHeights.heights[height]);
        this.targetY = AprilTagConstants.yPositions[closestNum] + shooter.getOffset() + (team == Alliance.Red ? AprilTagConstants.targetYOffsetRed : AprilTagConstants.targetYOffset);
        if(team == Alliance.Red) {
            this.targetY = AprilTagConstants.totalY - targetY;
        }
        this.goal = new Pose2d(targetX, targetY, converted);
        lights.setColor(255, 0, 0);
    }

    public void execute() {
        if(shooter.prepsByDefault()) {
            shooter.setup(TargetHeights.heights[height]);
        }
        super.execute();
    }

    public boolean isFinished() {
        return isFinished;
    }

    public void end(boolean interrupted) {
        lights.off();
        super.end(interrupted);
    }
}
