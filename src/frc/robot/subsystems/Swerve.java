package frc.robot.subsystems;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.modules.SwerveModule;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Util; 
public class Swerve extends SubsystemBase { // our swerve drive subsystem
    public enum AlignState {
        NOT,
        CONTROLLERON,
        CONTROLLEROFF
    }
    private Vision vision; // it has to be able to access the vision system for position tracking
    private SwerveDrivePoseEstimator poseEstimator; // specialized position estimator that uses motor data and vision data
    public SwerveModule[] drivers; // an array of 4 swerve modules for the four modules on the robot
    public Pigeon2 gyro; // keep a gyro to read our current angle
    public boolean usingVision;
    public boolean foundPosition; // keeps track of whether the robot has gotten an initial position reading from the vision system
    public boolean usingCones; // keep track of the current station (0-2) that the driver wants to go to based on the button board
    public int currentHeight; // keep track of whether the driver wants to shoot low, mid, or high based on the button board
    private ChassisSpeeds previousMove; // keep track of the previous speeds of the modules for position tracking
    public boolean slowMode; // whether the swerve drive is in slowmode
    public AlignState isAligning;
    public boolean autoShoot;
    public Swerve(Vision vision, boolean pigeon) { // the pigeon parameter tells the code whether we are using a pigeon
        autoShoot = true;
        isAligning = AlignState.NOT;
        drivers = new SwerveModule[SwerveDriveConstants.info.length]; // instantiate the module array
        usingVision = true;
        for(int i = 0; i < drivers.length; i++)  {
            drivers[i] = new SwerveModule(SwerveDriveConstants.info[i], i, true); // for each module, instantiate the module with predefined constant module info
        }
        slowMode = false; // slow mode starts off
        previousMove = new ChassisSpeeds(); // the last chassisspeeds were zero
        
        if(pigeon) { // if the gyro is used, set it up and set it to zero
            gyro = new Pigeon2(Ports.gyro);
            gyro.configFactoryDefault(100);
            gyro.configMountPose(90.0, 0.0, -2.4); // this configures our gyro pose so that we can read the pitch value
            gyro.setYaw(0.0);
        } else {
            gyro = null;
        }
        usingCones = false; // set default values for these (to be changed later by the codriver) 
        currentHeight = 1;
        this.vision = vision; // keep track of the vision system
        Rotation2d tation = Rotation2d.fromDegrees(angle()); // get our current gyro angle
        // instantiate the pose estimator based on our current angle and motor data, with a pose at the origin
        Matrix<N3, N1> stateDevs = new Matrix<N3,N1>(N3.instance, N1.instance);
            stateDevs.set(0, 0, 0.8);
            stateDevs.set(1, 0, 0.8);
            stateDevs.set(2, 0, 0.1);
        Matrix<N3, N1> visionDevs = new Matrix<N3,N1>(N3.instance, N1.instance);
            visionDevs.set(0, 0, 0.1);
            visionDevs.set(1, 0, 0.1);
            visionDevs.set(2, 0, 99.0);
        poseEstimator = new SwerveDrivePoseEstimator(SwerveDriveConstants.kinematics, tation, getPositions(), new Pose2d(0.0, 0.0, tation), stateDevs, visionDevs);
        foundPosition = !vision.usesCamera(); // if no camera, just start at zero and move from there. otherwise, wait until the camera reads a value to update the pose tracker

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain"); // add a special drivetrain tab to shuffleboard
        ShuffleboardLayout layout = tab.getLayout("Main", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 5); // add a main layout within the tab
         layout.addNumber("yaw", this::angle); // print all 3 euler angles of the gyro at all times for testing
        layout.addNumber("pitch", this.gyro::getPitch);
        // layout.addNumber("roll", () -> this.gyro.getRoll());
        //layout.addNumber("compass", () -> this.gyro.getCompassHeading()); // wtf is this lmao
        //layout.addBoolean("slow mode", () -> this.slowMode); // shows whether we are in slow mode
        //layout.add("reset gyro", new ResetGyro(this)); // add a button on the screen to reset the gyro to 0 degrees
        layout.addNumber("x position", () -> pose().getX()); // print out our x and y position
        layout.addNumber("y position", () -> pose().getY());
        layout.addNumber("forward m", () -> previousMove.vxMetersPerSecond); // print out our current speed and turning speed
        layout.addNumber("sideways m", () -> previousMove.vyMetersPerSecond);
        layout.addNumber("turning (rad)", () -> previousMove.omegaRadiansPerSecond);
        ShuffleboardTab selectionTab = Shuffleboard.getTab("Shot Selection");
        for(int i = 0; i < 4; i++) {
                int savedI = 3 - i;
                selectionTab.addBoolean("row " + i, () -> (currentHeight == savedI))
                    .withSize(1, 1)
                    .withPosition(0, i);
        }
        selectionTab.addBoolean("SHOOTING CONES", () -> usingCones)
        .withSize(2, 1).withPosition(6, 0);

        selectionTab.addBoolean("Auto Shoot", () -> autoShoot).withSize(2, 1).withPosition(6, 1);
        //selectionTab.addBoolean("Thinks is aligning", () -> isAligning != AlignState.NOT).withPosition(9, 3).withSize(1, 1);
    }
    private SwerveModulePosition[] getPositions() { // get the total lengths driven by each module as an array
        SwerveModulePosition[] arr = new SwerveModulePosition[drivers.length];
        for(int i = 0; i < drivers.length; i++) {
            arr[i] = drivers[i].getPosition(); // populate the array with each module position
        }
        return arr;
    }
    public SwerveModuleState[] getStates() { // get the current speed and angle of each module
        SwerveModuleState[] moduleState = new SwerveModuleState[drivers.length];
        for(int i = 0; i < moduleState.length; i++) {
            moduleState[i] = drivers[i].getState(); // populate the array with each module speed and angle
        }
        return moduleState;
    }
    public void resetOdometry(Pose2d pose) { // reset the odometry to a certain position using motor data and the current angle
        poseEstimator.resetPosition(Rotation2d.fromDegrees(this.angle()), getPositions(), pose);
    }
    public void periodic() {
        Rotation2d angle = Rotation2d.fromDegrees(angle()); // read the current angle of the robot
        SwerveModulePosition[] positions = getPositions(); // get all the module positions
        if(!foundPosition && usingVision && vision.usesCamera()) { // if we're still waiting for the first vision position reading, do this
            Pose2d measure = vision.updateRobotPose(poseEstimator, angle, new Pose2d(), false); // get the current vision measurement
            if(measure != null) {
                foundPosition = true;
                poseEstimator.resetPosition(angle, getPositions(), measure);
            }
        } else {
            // otherwise, do the normal stuff
            Pose2d pose = poseEstimator.update(angle, positions); // update our position estimator using the current lag time, the robot angle, and the module positions
            if(usingVision) {
                vision.updateRobotPose(poseEstimator, angle, pose, true); // try to get a reading from the vision system
            }
        }
    }
    public void drive(double forward, double left, double rotation, boolean fieldCentric, boolean pid) {
        //System.out.println("driving: " + forward + ", " + left + ", " + rotation);
        // drive the robot at a certain speed forward, a certain speed to the left, and rotating at a certain speed
        // if fieldCentric is false, this will drive forward/left from the perspective of the robot. otherwise, it uses the perspective of the field
        // if pid is true, it will use more accurate and brake-y control for the drive motors. this is true for auto but false for regular driving
        ChassisSpeeds speeds = (fieldCentric) ? ChassisSpeeds.fromFieldRelativeSpeeds(forward, left, rotation, Rotation2d.fromDegrees(this.angle())) : new ChassisSpeeds(forward, left, rotation);
        // ^ generate desired speeds for the chassis
        this.previousMove = speeds;
        SwerveModuleState[] states = SwerveDriveConstants.kinematics.toSwerveModuleStates(speeds); // convert the chassis speed to individual states for each module
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveDriveConstants.maxVelocity); // make sure that the states conform to our max velocity
        updateMotors(states, pid, forward == 0.0 && left == 0.0 && rotation == 0.0);
        // if the driver isn't touching the controller, don't just turn to 0 for no reason. instead, don't move at all
    }
    public Pose2d pose() { // get the current position based on the estimator, or 0 if the position hasn't been estimated yet
        return poseEstimator.getEstimatedPosition();
    }
    public double angle() { // get the yaw angle if we're using a gyro, and subtract the offset to adjust for when we zero the gyro
        return gyro == null ? 0.0 : (gyro.getYaw());
    }
    public void resetGyro(double angle) {
        gyro.setYaw(angle);
    }
    public double anglePitch() { // get the pitch angle of the gyro
        return gyro == null ? 0.0 : gyro.getPitch();
    }
    public void brake() { // turn all the motors inwards so that it makes our robot hard to move
        drivers[0].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)), false, false, true);
        drivers[1].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(315.0)), false, false, true);
        drivers[2].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0)), false, false, true);
        drivers[3].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(225.0)), false, false, true);
    }
    public void stop() { // stop the robot from moving
        this.drive(0.0, 0.0, 0.0, true, false);
    }
    public double getAverageSpeed() { // get the average absolute speed of the swerve modules, in meters per second
        double total = 0.0;
        for(int i = 0; i < drivers.length; i++) {
            total += Math.abs(Util.nativeUnitsToMetersPerSecond(drivers[i].getDrive())); // sum up all of the speeds and divide by the number of modules to get the average
        };
        return total / drivers.length;
    }
    public void updateMotors(SwerveModuleState[] myStates, boolean pid, boolean preventTurn) { // update each motor based on desired swerve module states
        for(int i = 0; i < drivers.length; i++) {
            drivers[i].setDesiredState(myStates[i], preventTurn, this.slowMode, pid); // iterate through the array and set each motor
        }
    }
}