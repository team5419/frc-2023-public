package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.Limelight;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.io.File;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
    private PhotonPoseEstimator poseEstimator; // the photon camera has its own pose estimator that interacts with the overall pose estimator
    public Team team; // keep track of the team that we think we're on
    private Rotation3d lastTagRotation;
    private Pose2d lastTagPosition;
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
        lastTagRotation = new Rotation3d(0.0, 0.0, 0.0);
        lastTagPosition = new Pose2d();
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
            layout.addDouble("Last tag yaw", () -> lastTagRotation.getZ());
            layout.addDouble("Last tag pitch", () -> lastTagRotation.getY());
            layout.addDouble("Last tag roll", () -> lastTagRotation.getX());

            layout.addDouble("Last tag x", () -> lastTagPosition.getX());
            layout.addDouble("Last tag y", () -> lastTagPosition.getY());
            camera = new PhotonCamera("back");
        } else {
            camera = null;
        }
        poseEstimator = null; // we'll create the pose estimator once we know what team we're on
    }

    public boolean usesCamera() {
        return camera != null;
    }

    private Team getTeam() {
        //System.out.println("getting team");
        if(camera == null) {
            return Team.NONE;
        }
        PhotonPipelineResult result = camera.getLatestResult();
        if(!result.hasTargets()) {
            return Team.NONE;
        }
        PhotonTrackedTarget best = result.getBestTarget();
        // System.out.println(best.getYaw());
        // System.out.println(best.getPitch());
        // System.out.println(best.getBestCameraToTarget());
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
        Pose2d prev = previous;
        if(team == Team.RED) {
            prev = new Pose2d(AprilTags.totalX - prev.getX(), prev.getY(), prev.getRotation());
        }
        poseEstimator.setReferencePose(prev);
        Optional<EstimatedRobotPose> result = poseEstimator.update();
        if (result.isPresent()) {
            lastTagRotation = result.get().estimatedPose.getRotation();
            Pose2d asPose = result.get().estimatedPose.toPose2d();
            if(team == Team.RED) {
                asPose = new Pose2d(AprilTags.totalX - asPose.getX(), asPose.getY(), asPose.getRotation());
            }
            lastTagPosition = asPose;
            return new Pair<Pose2d, Double>(asPose, result.get().timestampSeconds);
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
        // System.out.println("aaa");
        // System.out.println(team == Team.NONE);
        // System.out.println(team == Team.RED);
        // System.out.println(team == Team.BLUE);
        if(team == Team.NONE) {
            team = getTeam();
        } else if(poseEstimator == null && camera != null) {
            AprilTagFieldLayout tagLayout = null;
            try {
                //File file = new File(team == Team.RED ? "./aprilTagsRed.json" : "./aprilTagsBlue.json");
                tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
                
            } catch(Exception e) {
                System.out.println("Unable to load april tags file!!");
                System.out.println(e);
            }
            poseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, AprilTags.robotToCam);
        }
    }
}