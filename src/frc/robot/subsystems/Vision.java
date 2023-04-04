package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import java.util.ConcurrentModificationException;
import java.util.List;
import java.util.Optional;

public class Vision extends SubsystemBase { // this keeps track of our limelight and photon camera

    private NetworkTable limelight; // keep track of the limelight
    private ShuffleboardTab layout; // keep track of a shuffleboard layout for printing data
    private PhotonCamera[] cameras; // keep track of the photon camera (april tags stuff)
    private Pose2d lastTagPositionFront;
    private double lastTagRotation;
    private AprilTagFieldLayout tagLayout;
    public boolean seesTag;
    private double previousLimelightHorizontal;
    private double previousLimelightVertical;
    private static double filterPercent = 0.6;
    public Vision(ShuffleboardTab tab, boolean _limelight, boolean _photon) { // the boolean parameters tell the code if we're using limelight and photon vision
        layout = Shuffleboard.getTab("Vision"); // create a shuffleboard layout to print data
        if(_limelight) { // if we're using a limelight, set it up and add some values to shuffleboard
            limelight = NetworkTableInstance.getDefault().getTable("limelight");    
            limelight.getEntry("pipeline").setNumber(1.0);        
            //layout.addNumber("Hor Offset", () -> getHorizontalOffset());
            //layout.addNumber("Ver Offset", () -> getVerticalOffset());
            layout.addNumber("Distance", () -> getHorizontalDistance());
            //layout.addNumber("Lin Hor Offset", () -> getLinearHorizontalOffset(getHorizontalDistance()));
            layout.addBoolean("Sees target", () -> isTargetFound());
            layout.add("Limelight on", Commands.runOnce(() -> {
                this.on();
            }));
            layout.add("Limelight off", Commands.runOnce(() -> {
                this.off();
            }));
            this.off();
        } else {
            limelight = null;
        }
        seesTag = false;
        lastTagRotation = 0.0;
        previousLimelightHorizontal = 0.0;
        previousLimelightVertical = 0.0;
        lastTagPositionFront = new Pose2d();
        tagLayout = null;
        try {
            tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch(Exception e) {
            //System.out.println(e);
        }
        
        if(_photon) { 
            layout.addDouble("Last tag x", () -> lastTagPositionFront.getX());
            layout.addDouble("Last tag y", () -> lastTagPositionFront.getY());
            layout.addBoolean("sees tag", () -> seesTag);
            layout.addDouble("last tag theta", () -> lastTagRotation);
            // layout.addDouble("x before transform", () -> rawData.getX());
            // layout.addDouble("y before transform", () -> rawData.getY());
        cameras = new PhotonCamera[] { new PhotonCamera("top port") }; // MAKE SURE BACK IS FIRST
        } else {
            cameras = null;
        }
        //poseEstimator = null; // we'll create the pose estimator once we know what team we're on
    }

    public boolean usesCamera() {
        return cameras != null;
    }

    public double getHorizontalToTarget(int number) {
        if(cameras == null) {
            return 0.0;
        }
        seesTag = false;
        PhotonPipelineResult res = cameras[0].getLatestResult();
        
        if(res != null && res.hasTargets()) {   
            List<PhotonTrackedTarget> targets = res.getTargets();
            for(int i = 0; i < targets.size(); i++) {
                if(targets.get(i).getFiducialId() == number) {
                    seesTag = true;
                    //System.out.println("found target");
                    return targets.get(i).getBestCameraToTarget().getY();
                }
            }
        }
        return 0.0;
    }

    public Pose2d updateRobotPose(SwerveDrivePoseEstimator poser, Rotation2d theta, Pose2d previous, boolean foundPosition) {
        if(cameras == null) {
            return null;
        }
        Alliance team = team();
        if(team == Alliance.Invalid) {
            return null;
        }
        boolean seenTag = false;
        for(int i = 0; i < cameras.length; i++) {
            PhotonPipelineResult res = cameras[i].getLatestResult();
            if(res != null && res.hasTargets()) {
                PhotonTrackedTarget target = res.getBestTarget();
                if(target != null && target.getPoseAmbiguity() <= AprilTagConstants.ambiguityRequirement) {
                    if(!seenTag) {
                        seenTag = true;
                    }
                    Transform3d transform = target.getBestCameraToTarget();
                    Pose2d reference = AprilTagConstants.robotToCam[i];
                    //double theta = target.getYaw() * Math.PI / 180.0; use this to rely on apriltag angle instead of gyro
                    double _theta = theta.getRadians() - reference.getRotation().getRadians();
                    if(i == 1) {
                        lastTagRotation = _theta;
                    }
                    double preTransformX = transform.getX();
                    double preTransformY = transform.getY();
                    if(team == Alliance.Red) {
                        preTransformX = -preTransformX;
                        preTransformY = -preTransformY;
                    }
                    preTransformX -= reference.getX();
                    preTransformY -= reference.getY();
                    double transformedX = preTransformX * Math.cos(_theta) - preTransformY * Math.sin(_theta);
                    double transformedY = preTransformX * Math.sin(_theta) + preTransformY * Math.cos(_theta);
                    Optional<Pose3d> tagPose = tagLayout.getTagPose(target.getFiducialId());
                    if(tagPose.isPresent()) {
                        Pose2d pose2d = new Pose2d(transformedX + tagPose.get().getX(), transformedY + tagPose.get().getY(), theta);
                        if(team == Alliance.Red) {
                            pose2d = new Pose2d(AprilTagConstants.totalX - pose2d.getX(), AprilTagConstants.totalY - pose2d.getY(), theta);
                        }
                        if(i == 1) {
                            lastTagPositionFront = pose2d;
                            lastTagRotation = target.getYaw();
                        }
                        if(!foundPosition) {
                            return pose2d;
                        }
                        try {
                            poser.addVisionMeasurement(pose2d, res.getTimestampSeconds());
                        } catch(ConcurrentModificationException ex) {
                            //System.out.println(ex.getMessage());
                        }
                    }
                }
            }
        }
        seesTag = seenTag;
        return null;
    }

    public double getHorizontalOffset() { // in settings, make sure limelight is filtering for lowest target closest to the middle
        double val = limelight == null ? 0.0 : limelight.getEntry("tx").getDouble(0.0);
        if(!isTargetFound()) {
            return previousLimelightHorizontal;
        }
        previousLimelightHorizontal = filterPercent * previousLimelightHorizontal + (1.0 - filterPercent) * val;
        return previousLimelightHorizontal;
    }

    public double getVerticalOffset() {
        double val = limelight == null ? 0.0 : limelight.getEntry("ty").getDouble(0.0);
        if(!isTargetFound()) {
            return previousLimelightVertical;
        }
        previousLimelightVertical = filterPercent * previousLimelightVertical + (1.0 - filterPercent) * val;
        return previousLimelightVertical;
    }

    public double getLinearHorizontalOffset(double dist) {
        if(limelight == null) {
            return 0.0;
        }
        double horOffset = getHorizontalOffset();
        //double dist = getHorizontalDistance();
        return Math.tan(Math.toRadians(horOffset)) * dist;
    }

    public double getHorizontalDistance() { 
        if(limelight == null) {
            return 0.0;
        }
        double offset = getVerticalOffset();
        if(LimelightConstants.cameraAngle + offset == 0.0) {
            return 0.0;
        }
        return (LimelightConstants.lowTargetHeight - LimelightConstants.cameraHeight) / Math.tan(Math.toRadians(LimelightConstants.cameraAngle + getVerticalOffset()));
    }

    // check if the limelight is picking up on the target
    public boolean isTargetFound() {
        return limelight != null && limelight.getEntry("tv").getDouble(0.0) > 0.0;
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

    public Alliance team() {
        return DriverStation.getAlliance();
    }

    public void periodic() {
    }
}