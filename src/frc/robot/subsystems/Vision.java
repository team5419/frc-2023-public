package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.Limelight;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants.AprilTags;
import java.util.ArrayList;
import java.util.Optional;

public class Vision extends SubsystemBase {
    public enum Team {
        NONE, BLUE, RED
    };
    private NetworkTable limelight;
    private ShuffleboardLayout layout;
    private AprilTagFieldLayout tagLayout;
    private PhotonCamera camera;
    private RobotPoseEstimator poseEstimator;
    public Team team;
    public Vision(ShuffleboardTab tab) {
        team = Team.NONE;
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        layout = tab.getLayout("Vision", BuiltInLayouts.kList).withPosition(3, 0).withSize(2, 4);
        layout.addNumber("Offset", () -> { return getHorizontalOffset(); });
        layout.addBoolean("Sees target", () -> { return isTargetFound(); });
        layout.addString("Team", () -> {
            switch(team) {
                case RED:
                    return "Red";
                case BLUE:
                    return "Blue";
                default:
                    return "None";
            }
        });
        try {
            tagLayout = AprilTagFieldLayout.loadFromResource("aprilTags.json");
        } catch(Exception e) {
            System.out.println("Unable to load april tags file!!");
            tagLayout = null;
        }
        camera = new PhotonCamera("photonvision");
        ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        camList.add(new Pair<PhotonCamera, Transform3d>(camera, AprilTags.robotToCam));
        poseEstimator = new RobotPoseEstimator(tagLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
    }

    private Team getTeam() {
        PhotonPipelineResult result = camera.getLatestResult();
        if(!result.hasTargets()) {
            return Team.NONE;
        }
        PhotonTrackedTarget best = result.getBestTarget();
        int id = best.getFiducialId();
        if(id < 1 || id > 8) {
            return Team.NONE;
        }
        if(id >= 5) {
            return Team.BLUE;
        }
        return Team.RED;
    }

    public Pair<Pose2d, Double> getRobotPose(Pose2d previous) {
        poseEstimator.setReferencePose(previous);
        double time = Timer.getFPGATimestamp();
        Optional<Pair<Pose3d, Double>> result = poseEstimator.update();
        if (result.isPresent()) {
            return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), time - result.get().getSecond());
        } else {
            return null;
        }
    }

    public double getHorizontalOffset() {
        return limelight.getEntry("tx").getDouble(0.0);
    }

    public double getVerticalOffset() {
        return limelight.getEntry("ty").getDouble(0.0);
    }

    public double getHorizontalDistance() { 
        return (Limelight.lowTargetHeight - Limelight.cameraHeight) / Math.tan(Math.toRadians(Limelight.cameraAngle + getVerticalOffset()));
    }

    // check if the limelight is picking up on the target
    public boolean isTargetFound() {
        return limelight.getEntry("tv").getDouble(0.0) > 0.0 && getVerticalOffset() != 0.0;
    }

    public void on() {
        limelight.getEntry("ledMode").setNumber(3);
    }

    public void off() {
        limelight.getEntry("ledMode").setNumber(1);
    }

    public void periodic() {
        if(team == Team.NONE) {
            team = getTeam();
        }
    }
}