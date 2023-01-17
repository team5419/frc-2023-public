package frc.robot;

import frc.robot.modules.ModuleInfo;
import frc.robot.classes.PID;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import java.lang.Math;

public class Constants {
    public static final class ProtoDrive {
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
    public static final class Drive {
        public static final PID DrivePID = new PID(0.1, 0.0, 0.0);
        public static final PID TurnPID = new PID(8.0 / 12.0, 0.0, 0.0);
        public static final double slow = 0.2;
        public static final double maxVelocity = 4.0;  
        public static final double kv = 2.298;//(2.1737 / 12); // 2.298
        public static final double ka = 0.17118;//(0.29281 / 12); // 0.17118
        public static final double ks = 0.10352;//(0.63566 / 12); // 0.10352
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

        public static final PIDController balanceController = new PIDController(1.0, 0.0, 0.0);
        public static final PIDController yawBalanceController = new PIDController(1.0, 0.0, 0.0);
        public static final double epsilonBalance = 1.0;
        public static final double epsilonYawBalance = 1.0;

        public static final double speedMultiplier = 4.0;
        public static final double turnMultiplier = 2.5;
        public static final double controllerDeadband = 0.1;

        public static final double pXY = 0.27;
        public static final double pTheta = 0.4;
        public static final double epsilonXY = 0.1;
        public static final double epsilonTheta = 1.0;
        public static final ModuleInfo[] info = {
            new ModuleInfo(1, 2, true, true, 9, 3.48213), //numbers are placeholders
            new ModuleInfo(7, 8, false, true, 12, 0.35895),
            new ModuleInfo(3, 4, true, true, 10, 1.2149),
            new ModuleInfo(5, 6, false, true, 11, 0.91578)
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
    public static final class Limelight {
        public static final double lowTargetHeight = 1.0;
        public static final double cameraAngle = 0.0;
        public static final double cameraHeight = 0.5;
        public static final PIDController horizontalPID = new PIDController(1.0, 0.0, 0.0);
        public static final PIDController turnPID = new PIDController(1.0, 0.0, 0.0);
        public static final PIDController forwardPID = new PIDController(1.0, 0.0, 0.0);
        public static final double desiredAngle = 0.0;
        public static final double desiredDistance = 0.0;

        public static final double epsilonTurn = 1.0;
        public static final double epsilonHorizontal = 0.1;
        public static final double epsilonForward = 0.1;
    }
    public static final class AprilTags {
        public static Transform3d robotToCam = new Transform3d( // we can prob just leave this at 0 and use camera as a reference to our robot 
            new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, Math.PI)); // however we should change angle to keep in accordance with gyro angle
            
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
        public static double[] cubeDists = {
            1.0, // high
            1.0, // mid
            1.0 // low
        };
        public static double[] coneDists = {
            1.0, // high
            1.0, // mid
            1.0 // low
        };
        public static double cubeRotation = 0.0;
        public static double coneRotation = 0.0;
        public static final double xPosBeforeBarriers = 1.5; // to be measured
        public static final double xEndOfChargingStation = 3.5; // to be measured
    }
    public static final class Ports {
        public static final int intake = 14;
        public static final int leftLeader = 1;
        public static final int leftFollower = 2;
        public static final int rightLeader = 3;
        public static final int rightFollower = 4;
        public static final int gyro = 13;
        public static final int claw = 15;
    }
};