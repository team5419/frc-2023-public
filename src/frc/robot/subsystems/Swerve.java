package frc.robot.subsystems;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.modules.SwerveModule;

public class Swerve extends SubsystemBase {
    private Vision vision;
    private SwerveDrivePoseEstimator poseEstimator;
    private SwerveModule[] drivers;
    private boolean foundPosition;
    public int currentNum;
    public int currentHeight;
    public Swerve(Vision vision) {
        currentNum = 0;
        currentHeight = 0;
        this.vision = vision;
        poseEstimator = new SwerveDrivePoseEstimator(/* TBM */null, Rotation2d.fromDegrees(angle()), getPositions(), new Pose2d());
        drivers = /* TBM */ new SwerveModule[0];
        foundPosition = false;
    }
    private SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] arr = new SwerveModulePosition[drivers.length];
        for(int i = 0; i < drivers.length; i++) {
            arr[i] = drivers[i].getPosition();
        }
        return arr;
    }
    public void periodic() {
        Rotation2d angle = Rotation2d.fromDegrees(angle());
        SwerveModulePosition[] positions = getPositions();
        if(!foundPosition) {
            Pair<Pose2d, Double> initialMeasurement = vision.getRobotPose(new Pose2d());
            if(initialMeasurement != null) {
                foundPosition = true;
                poseEstimator.resetPosition(angle, positions, initialMeasurement.getFirst());
            }
        } else {
            Pose2d pose = poseEstimator.updateWithTime(Timer.getFPGATimestamp(), angle, positions);
            Pair<Pose2d, Double> res = vision.getRobotPose(pose);
            if(res != null) {
                poseEstimator.addVisionMeasurement(res.getFirst(), res.getSecond());
            }
        }
    }
    public void drive(double forward, double left, double turn) {
        // TBD
    }
    public Pose2d pose() {
        return foundPosition ? poseEstimator.getEstimatedPosition() : new Pose2d();
    }
    public double angle() {
        return 0.0;
    }
    public double anglePitch() {
        return 0.0;
    }
    public void brake() {
        // TBD
    }
    public double getAverageSpeed() {
        // TBM
        return 0.0;
    }
}
