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

public class Vision extends SubsystemBase { // this keeps track of our limelight and photon camera
    public enum Team { // whether vision thinks we are on red team or blue team (starts out as Team.NONE until one is found)
        NONE, BLUE, RED
    };
    private NetworkTable limelight; // keep track of the limelight
    private ShuffleboardLayout layout; // keep track of a shuffleboard layout for printing data
    private PhotonCamera camera; // keep track of the photon camera (april tags stuff)
    private RobotPoseEstimator poseEstimator; // the photon camera has its own pose estimator that interacts with the overall pose estimator
    public Team team; // keep track of the team that we think we're on
    public Vision(ShuffleboardTab tab, boolean _limelight, boolean _photon) { // the boolean parameters tell the code if we're using limelight and photon vision
        team = Team.NONE; // we don't know what team we're on yet
        layout = tab.getLayout("Vision", BuiltInLayouts.kList).withPosition(1, 0).withSize(2, 4); // create a shuffleboard layout to print data
        if(_limelight) { // if we're using a limelight, set it up and add some values to shuffleboard
            limelight = NetworkTableInstance.getDefault().getTable("limelight");            
            layout.addNumber("Offset", () -> getHorizontalOffset());
            layout.addBoolean("Sees target", () -> isTargetFound());
        } else {
            limelight = null;
        }
        if(_photon) { // if we're using photon camera, set it up and add a value to shuffleboard for what team we're on
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
            camera = new PhotonCamera("photonvision");
            ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
            camList.add(new Pair<PhotonCamera, Transform3d>(camera, AprilTags.robotToCam));
        } else {
            camera = null;
        }
        poseEstimator = null; // we'll create the pose estimator once we know what team we're on
    }

    public boolean usesCamera() {
        return camera != null;
    }

    private Team getTeam() {
        if(camera == null) {
            return Team.NONE;
        }
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
        if(poseEstimator == null) {
            return null;
        }
        poseEstimator.setReferencePose(previous);
        double time = Timer.getFPGATimestamp();
        Optional<Pair<Pose3d, Double>> result = poseEstimator.update();
        if (result.isPresent()) {
            return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), time - result.get().getSecond());
        } else {
            return null;
        }
    }

    public double getHorizontalOffset() { // in settings, make sure limelight is filtering for lowest target closest to the middle
        return limelight == null ? 0.0 : limelight.getEntry("tx").getDouble(0.0);
    }

    public double getVerticalOffset() {
        return limelight == null ? 0.0 : limelight.getEntry("ty").getDouble(0.0);
    }

    public double getHorizontalDistance() { 
        if(limelight == null) {
            return 0.0;
        }
        return (Limelight.lowTargetHeight - Limelight.cameraHeight) / Math.tan(Math.toRadians(Limelight.cameraAngle + getVerticalOffset()));
    }

    // check if the limelight is picking up on the target
    public boolean isTargetFound() {
        return limelight != null && limelight.getEntry("tv").getDouble(0.0) > 0.0 && getVerticalOffset() != 0.0;
    }

    public void on() {
        if(limelight == null) {
            return;
        }
        limelight.getEntry("ledMode").setNumber(3);
    }

    public void off() {
        if(limelight == null) {
            return;
        }
        limelight.getEntry("ledMode").setNumber(1);
    }

    public void periodic() {
        if(team == Team.NONE) {
            team = getTeam();
        } else if(poseEstimator == null && camera != null) {
            AprilTagFieldLayout tagLayout = null;
            try {
                tagLayout = AprilTagFieldLayout.loadFromResource(team == Team.RED ? "aprilTagsRed.json" : "aprilTagsBlue.json");
            } catch(Exception e) {
                System.out.println("Unable to load april tags file!!");
            }
            ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
            camList.add(new Pair<PhotonCamera, Transform3d>(camera, AprilTags.robotToCam));
            poseEstimator = new RobotPoseEstimator(tagLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
        }
    }
}