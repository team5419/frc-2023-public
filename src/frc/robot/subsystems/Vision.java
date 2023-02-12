package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

import java.util.ConcurrentModificationException;
import java.util.Optional;

public class Vision extends SubsystemBase { // this keeps track of our limelight and photon camera
    public enum Team { // whether vision thinks we are on red team or blue team (starts out as Team.NONE until one is found)
        NONE, BLUE, RED
    };
    private NetworkTable limelight; // keep track of the limelight
    private ShuffleboardLayout layout; // keep track of a shuffleboard layout for printing data
    private PhotonCamera[] cameras; // keep track of the photon camera (april tags stuff)
    //private PhotonPoseEstimator[] poseEstimator; // the photon camera has its own pose estimator that interacts with the overall pose estimator
    public Team team; // keep track of the team that we think we're on
    private Pose2d lastTagPositionFront;
    private Translation2d rawData;
    private double lastTagRotation;
    private AprilTagFieldLayout tagLayout;
    public boolean seesTag;
    public Vision(ShuffleboardTab tab, boolean _limelight, boolean _photon) { // the boolean parameters tell the code if we're using limelight and photon vision
        team = Team.NONE; // we don't know what team we're on yet
        layout = tab.getLayout("Vision", BuiltInLayouts.kList).withPosition(0, 1).withSize(2, 4); // create a shuffleboard layout to print data
        if(_limelight) { // if we're using a limelight, set it up and add some values to shuffleboard
            limelight = NetworkTableInstance.getDefault().getTable("limelight");            
            layout.addNumber("Offset", () -> getHorizontalOffset());
            layout.addBoolean("Sees target", () -> isTargetFound());
        } else {
            limelight = null;
        }
        seesTag = false;
        lastTagRotation = 0.0;
        lastTagPositionFront = new Pose2d();
        rawData = new Translation2d();
        tagLayout = null;
        try {
            tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch(Exception e) {
            System.out.println(e);
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
            layout.addDouble("Last tag x", () -> lastTagPositionFront.getX());
            layout.addDouble("Last tag y", () -> lastTagPositionFront.getY());
            layout.addBoolean("sees tag", () -> seesTag);
            // layout.addDouble("last tag theta", () -> lastTagRotation);
            // layout.addDouble("x before transform", () -> rawData.getX());
            // layout.addDouble("y before transform", () -> rawData.getY());
        cameras = new PhotonCamera[] { new PhotonCamera("back lifecam")/* , new PhotonCamera("front") */}; // MAKE SURE BACK IS FIRST
        } else {
            cameras = null;
        }
        //poseEstimator = null; // we'll create the pose estimator once we know what team we're on
    }

    public boolean usesCamera() {
        return cameras != null;
    }

    private Team getTeam() {
        //System.out.println("getting team");
        if(cameras == null) {
            return Team.NONE;
        }
        PhotonPipelineResult result = cameras[0].getLatestResult(); // first camera is the back
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

    public Pose2d updateRobotPose(SwerveDrivePoseEstimator poser, Rotation2d theta, Pose2d previous, boolean foundPosition) {
        if(team == Team.NONE) {
            return null;
        }
        for(int i = 0; i < cameras.length; i++) {
            PhotonPipelineResult res = cameras[i].getLatestResult();
            seesTag = res != null && res.hasTargets();
            if(seesTag) {
                PhotonTrackedTarget target = res.getBestTarget();
                seesTag = target != null && target.getPoseAmbiguity() <= AprilTagConstants.ambiguityRequirement;
                if(seesTag) {
                    Transform3d transform = target.getBestCameraToTarget();
                    Pose2d reference = AprilTagConstants.robotToCam[i];
                    //double theta = target.getYaw() * Math.PI / 180.0; use this to rely on apriltag angle instead of gyro
                    double _theta = theta.getRadians() - reference.getRotation().getRadians();
                    if(i == 0) {
                        lastTagRotation = _theta;
                    }
                    double preTransformX = transform.getX() - reference.getX();
                    double preTransformY = transform.getY() - reference.getY();
                    double transformedX = preTransformX * Math.cos(_theta) - preTransformY * Math.sin(_theta);
                    double transformedY = preTransformX * Math.sin(_theta) + preTransformY * Math.cos(_theta);
                    Optional<Pose3d> tagPose = tagLayout.getTagPose(target.getFiducialId());
                    Pose2d pose2d = new Pose2d(transformedX + tagPose.get().getX(), transformedY + tagPose.get().getY(), theta);
                    if(team == Team.RED) {
                        pose2d = new Pose2d(AprilTagConstants.totalX - pose2d.getX(), AprilTagConstants.totalY - pose2d.getY(), theta);
                    }
                    if(i == 0) {
                        lastTagPositionFront = pose2d;
                        rawData = new Translation2d(transform.getX(), transform.getY());
                        //lastTagPositionFront = new Pose2d(transform.getX(), transform.getY(), new Rotation2d(0.0)); // actually this is the back reading
                    }
                    if(!foundPosition) {
                        return pose2d;
                    }
                    try {
                        poser.addVisionMeasurement(pose2d, res.getTimestampSeconds());
                    } catch(ConcurrentModificationException ex) {
                        System.out.println(ex.getMessage());
                    }
                    
                }
            }
        }
        return null;
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
        return (LimelightConstants.lowTargetHeight - LimelightConstants.cameraHeight) / Math.tan(Math.toRadians(LimelightConstants.cameraAngle + getVerticalOffset()));
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
        }
    }
}