package frc.robot;

import frc.robot.modules.ModuleInfo;
import frc.robot.classes.PID;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import java.lang.Math;
import java.util.Map;

public class Constants {
    public static enum ConerTypes {
        LOW_CONER, EVERYBOT_MOTORS, EVERYBOT_SUCTION
    }
    public static final class AutoConstants { // all relative to where you start, where positive y and positive x are towards the charge station
        public static final Translation2d firstCube = new Translation2d(4.3, 0.35); // 0.5
        public static final Translation2d firstShot = new Translation2d(2.9, 0.3); // 0.5
        public static final Translation2d firstShotShortSide = new Translation2d(2.95, 0.6); // 1.05
        public static final Translation2d preSecondCube = new Translation2d(3.4, 1.45);
        public static final Translation2d secondCube = new Translation2d(4.3, 1.45); // 1.5
        public static final Translation2d secondShot = new Translation2d(3.1, 1.95); // 2.25
        public static final Translation2d preBalancePosition = new Translation2d(2.4, 1.95); // 2.25
    }
    public static final class TargetHeights { // these need to be stored as strings for shuffleboard
        public static final String LOW = "LOW";
        public static final String MID = "MID";
        public static final String HIGH = "HIGH";
        public static final String INTAKE = "INTAKE";
        public static final String FAR = "FAR";
        public static final String[] heights = { LOW, MID, HIGH, FAR };
    }
    public static final class ConerConstants {
        public static final double inOutVelocity = 0.05;
        // order: bottom motor, top motor
        
    }
    public static final class CubeShooterConstants {

        public static final PID upPID = new PID(0.1, 0.0007, 0.0, 0.00);//new PID(0.18, 0.0001, 0.0, 0.00);
       
        // order: indexer, main motor

        public static final Map<String, Double> measuredVelocities = Map.of(TargetHeights.LOW, 0.0, TargetHeights.MID, 1500.0, TargetHeights.HIGH, /*1850.0,*/ 2800.0, TargetHeights.INTAKE, 0.0, TargetHeights.FAR, 1770.0);

        public static final double indexerSlowBackwardsSpeed = -0.05;
        public static final double sensorThresholdLeft = 880.0;
        public static final double sensorThresholdRight = 2170.0;

        public static final double adjustmentLeft = -0.125;
        public static final double adjustmentRight = 0.1;
    }
    public static final class DifferentialDriveConstants {
        public static final double trackWidth = 1.781;
        public static final double maxAcceleration = 1.5;
        public static final double maxVelocity = 3.0;
        public static final double maxVoltage = 12.0;
        public static final double kv = 2.3;
        public static final double ka = 0.463;
        public static final double ks = 0.191;
        public static final double maxCentripetalAcceleration = 3.0;
        public static final double beta = 2.0;
        public static final double zeta = 0.7;
        public static final double stupidTurn = 0.3;
        public static final double stupidThrottle = 0.1;
        public static final double ticksPerRotation = 2048.0 * 10.3333;
    };
    public static final class SwerveDriveConstants {
        public static final double triggerDeadband = 0.5;
        public static final int dPadInputRange = 30;
        public static final PID DrivePID = new PID(0.1, 0.0, 0.0);
        public static final PID TurnPID = new PID(8.0 / 12.0, 0.0, 0.0); // 8/12
        public static final double slow = 0.2;


        public static final double maxVelocity = 4.0; 
        public static final double maxAcceleration = 3.3;
        public static final PIDController xController = new PIDController(1.0, 0.0, 0.0);
        public static final PIDController yController = new PIDController(1.0, 0.0, 0.0);
        public static final double maxAngularSpeed = 1.2 * Math.PI;
        public static final TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(maxAngularSpeed, Math.pow(maxAngularSpeed, 2.0));
        public static final ProfiledPIDController thetaController = new ProfiledPIDController(10.0, 0.0, 0.0, thetaConstraints);

        public static final double kv = (2.298 / 12); // 2.1737
        public static final double ka = (0.17118 / 12); // 0.29281
        public static final double ks = (0.10352 / 12); // 0.63566
        public static final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(ks, kv, ka);
        public static final PIDController turnController = new PIDController(TurnPID.p, TurnPID.i, TurnPID.d);

        public static final double driveGearRatio = 6.75;
        public static final double turnGearRatio = 12.8;
        public static final double driveCPR = 2048.0;
        public static final double ticksPerRotation = (driveCPR * driveGearRatio);
        public static final double turnTicksPerRotation = (driveCPR * turnGearRatio);
        public static final double wheelRadius = 0.0508; // m
        public static final double wheelDiameter = wheelRadius * 2.0; // m
        public static final double wheelCircumference = wheelDiameter * Math.PI; // m
        public static final double regularBalanceP = 0.02;
        public static final double slowBalanceP = 0.02;
        public static final double constantBalanceSpeed = 0.15;
        public static final PIDController yawBalanceController = new PIDController(0.04, 0.0, 0.0);
        public static final double epsilonBalance = 1.0;
        public static final double epsilonYawBalance = 2.0;

        public static final double speedMultiplier = 4.0;
        public static final double turnMultiplier = 4.0;
        public static final double controllerDeadband = 0.1;

        public static final double pXY = 3.5;
        public static final double pTheta = 5.3;
        public static final double epsilonXY = 0.0254;
        public static final double epsilonTheta = 1.5;
        public static final ModuleInfo[] info = {
            new ModuleInfo(7, 8, true, true, 12, 1.97116132825613 -0.007669888436794), // 1.6229
            new ModuleInfo(5, 6, false, true, 11, 5.638901978731155  + 0.016873754560947), //4.7247            
            new ModuleInfo(1, 2, true, true, 9, 1.87758868932724 -0.006135910749435), //numbers are placeholders -1.6521
            new ModuleInfo(3, 4, false, true, 10, 2.900751806795597 + 0.047553308308125) // 1.6230
            
        };

        public static final Translation2d frontLeft = new Translation2d(0.2794, 0.2794);
        public static final Translation2d frontRight = new Translation2d(0.2794, -0.2794);
        public static final Translation2d backLeft = new Translation2d(-0.2794, 0.2794);
        public static final Translation2d backRight = new Translation2d(-0.2794, -0.2794);

        public static final Translation2d[] modulePositions = {frontLeft, frontRight, backLeft, backRight};

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeft, frontRight, backLeft, backRight);
        public static final double unitsPerMeter = 1.2511;
    };
    public static final class ClawConstants {
        public static final PID PID = new PID(1.0, 0.0, 0.0);
        public static final double openPosition = 0.0;
        public static final double closedPosition = 1.0;
    }
    public static final class LimelightConstants {
        public static final double lowTargetHeight = 0.6096;
        public static final double cameraAngle = 18.0;
        public static final double cameraHeight = 0.1778;
        public static final PIDController horizontalPID = new PIDController(0.08, 0.0, 0.0); // .08
        public static final PIDController linHorizontalPID = new PIDController(3.2, 0.0, 0.0); // 6.0
        public static final double closeConstant = 0.1;
        public static final PIDController turnPID = new PIDController(0.065, 0.0, 0.0); // .065
        public static final PIDController forwardPID = new PIDController(1.65, 0.0, 0.0); //6.2
        public static final double desiredAngle = 180.0;
        public static final double desiredDistance = 0.0;
        public static final double horizontalOffset = -1.65; // 1.65
        public static final double linHorizontalOffset = 0.02; //0.166 before theo left 0.008; // -0.023
        public static final double linHorizontalOffsetHigh = -0.02;


        public static final double epsilonTurn = 1.0;
        public static final double epsilonHorizontal = 0.5;
        public static final double epsilonLinHorizontal = 0.007;
        public static final double epsilonForward = 0.01; // .005

        public static final double limelightYawAngle = 11.5 * Math.PI / 180.0;
        public static final double cosYaw = Math.cos(limelightYawAngle);
        public static final double sinYaw = Math.sin(limelightYawAngle);
    }
    public static final class AprilTagConstants {

        public static final double cameraAngle = 0.0;

        public static Pose2d[] robotToCam = new Pose2d[] { 
            new Pose2d( // we can prob just leave this at 0 and use camera as a reference to our robot 
                -0.04, 0.0762, new Rotation2d(0.0)), // however we should change angle to keep in accordance with gyro angle
            new Pose2d(
                0.1, -0.20955, Rotation2d.fromDegrees(180.0))
        };
        public static double totalX = 16.54175;
        public static double totalY = 8.0137;
        public static double[] yPositions = {
            0.5128, // bottom pole
            1.0716, // bottom cube (furthest from HP)
            1.6304, // second pole
            2.1892, // third pole
            2.7480, // middle cube
            3.3068, // fourth pole
            3.8656, // fifth pole
            4.4244, // top cube
            4.9832 // top pole
        };

            public static final double targetYOffset = 2.549 - 2.748;
        public static final double xPosBeforeBarriers = 1.6; // to be measured
        public static final double xEndOfChargingStation = 2.55; // to be measured
        public static final double yOutsideRightChargingStation = -1.0;
        public static final double yInsideRightChargingStation = -1.0;
        public static final double yInsideLeftChargingStation = 1000.0;
        public static final double yOutsideLeftChargingStation = 1000.0;
        public static final double ambiguityRequirement = 0.9;
    }
    public static final class SensorArrayConstants {
        public static final int numSensors = 8;
        public static final int[] sensorOffsets = {-135, -90, -45, -15, 15, 45, 90, 135}; //mm
        public static final int baud = 115200;
    }

    public static final class Ports {
        public static final int intake = 18;
        public static final int indexer = 14;
        public static final int leftLeader = 1;
        public static final int leftFollower = 2;
        public static final int rightLeader = 3;
        public static final int rightFollower = 4;
        public static final int gyro = 13;
        public static final int claw = 15;
        public static final int coneTop = 17;
        public static final int coneBottom = 16;
        public static final int everyIntakeMotor = 19;
        public static final int suctionMotor = 22;
        public static final int cuberSolenoidA = 7;
        public static final int cuberSolenoidB = 0;
        public static final int conerSolenoidA = 4;
        public static final int conerSolenoidB = 5;
        public static final int solenoidC = 1;
        public static final int lifter = 24;
        public static final int lifterCancoder = 25;
        public static final int cuberSensor = 3;
        public static final int candlePort = 23;
        public static final int elevatorPort = 26;
    }
};