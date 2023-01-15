package frc.robot.subsystems;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.modules.SwerveModule;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Ports;
import frc.robot.Util; 
import frc.robot.commands.ResetGyro;

public class Swerve extends SubsystemBase {
    private Vision vision;
    private SwerveDrivePoseEstimator poseEstimator;
    public SwerveModule[] drivers = {
        new SwerveModule(Drive.info[0], 0),
        new SwerveModule(Drive.info[1], 1),
        new SwerveModule(Drive.info[2], 2),
        new SwerveModule(Drive.info[3], 3)
     };
    public Pigeon2 gyro;
    private boolean foundPosition;
    public int currentNum;
    public int currentHeight;
    private ChassisSpeeds previousMove;
    public boolean slowMode;
    public Swerve(Vision vision) {
        slowMode = false;
        previousMove = new ChassisSpeeds();
        gyro = new Pigeon2(Ports.gyro);
        gyro.configFactoryDefault(100);
        gyro.setYaw(0.0, 100);
        currentNum = 0;
        currentHeight = 0;
        this.vision = vision;
        poseEstimator = new SwerveDrivePoseEstimator(/* TBM */null, Rotation2d.fromDegrees(angle()), getPositions(), new Pose2d());
        foundPosition = false;

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        ShuffleboardLayout layout = tab.getLayout("Main", BuiltInLayouts.kList).withPosition(0, 0).withSize(1, 5);
        layout.addNumber("gyro", () -> this.angle());
        layout.addBoolean("slow mode", () -> this.slowMode);
        layout.add("reset gyro", new ResetGyro(this));
        layout.addNumber("x position", () -> pose().getX());
        layout.addNumber("y position", () -> pose().getY());
        layout.addNumber("forward m/s", () -> previousMove.vxMetersPerSecond);
        layout.addNumber("sideways m/s", () -> previousMove.vyMetersPerSecond);
        layout.addNumber("turning (rad)", () -> previousMove.omegaRadiansPerSecond);
    }
    private SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] arr = new SwerveModulePosition[drivers.length];
        for(int i = 0; i < drivers.length; i++) {
            arr[i] = drivers[i].getPosition();
        }
        return arr;
    }
    public SwerveModuleState[] getStates() {
        SwerveModuleState[] moduleState = new SwerveModuleState[drivers.length];
        for(int i = 0; i < moduleState.length; i++) {
            moduleState[i] = drivers[i].getState();
        }
        return moduleState;
    }
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(Rotation2d.fromDegrees(this.angle()), getPositions(), pose);
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
    public void drive(double forward, double left, double rotation, boolean fieldCentric, boolean pid) {
        ChassisSpeeds speeds = (fieldCentric) ? ChassisSpeeds.fromFieldRelativeSpeeds(forward, left, rotation, Rotation2d.fromDegrees(this.angle())) : new ChassisSpeeds(forward, left, rotation);
        this.previousMove = speeds;
        SwerveModuleState[] states = Drive.kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Drive.maxVelocity);
        updateMotors(states, pid, forward == 0.0 && left == 0.0 && rotation == 0.0);
    }
    public Pose2d pose() {
        return foundPosition ? poseEstimator.getEstimatedPosition() : new Pose2d();
    }
    public double angle() {
        return gyro.getYaw();
    }
    public double anglePitch() {
        return gyro.getPitch();
    }
    public void brake() {
        drivers[0].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)), false, false, true);
        drivers[1].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(315.0)), false, false, true);
        drivers[2].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0)), false, false, true);
        drivers[3].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(225.0)), false, false, true);
    }
    public void stop() {
        this.drive(0.0, 0.0, 0.0, true, false);
    }
    public double getAverageSpeed() {
        double total = 0.0;
        for(int i = 0; i < drivers.length; i++) {
            total += Util.nativeUnitsToMetersPerSecond(drivers[i].getDrive());
        };
        return total / drivers.length;
    }
    private void updateMotors(SwerveModuleState[] myStates, boolean pid, boolean preventTurn) {
        for(int i = 0; i < drivers.length; i++) {
            drivers[i].setDesiredState(myStates[i], preventTurn, this.slowMode, pid);
        }
    }
}