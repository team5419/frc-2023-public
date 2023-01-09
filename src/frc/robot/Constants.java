package frc.robot;
import frc.robot.classes.PID;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.controller.PIDController;
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
    };
    public static final class Drive {
        public static final PID DrivePID = new PID(0.1, 0.0, 0.0);
        public static final PID TurnPID = new PID(8.0, 0.0, 0.0);
        public static final double slow = 0.2;
        public static final double maxVelocity = 4.0;
        public static final double kv = (2.1737 / 12);
        public static final double ka = (0.29281 / 12);
        public static final double ks = (0.63566 / 12);
        public static final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(ks, kv, ka);
        public static final PIDController turnController = new PIDController(TurnPID.p, TurnPID.i, TurnPID.d);

        public static final double driveGearRatio = 6.75;
        public static final double driveCPR = 2048.0;
        public static final double ticksPerRotation = (driveCPR * driveGearRatio);
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
        public static Transform3d robotToCam = new Transform3d(
            new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0));
        public static double[] xPositions = {
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
        public static PIDController horizontalPID = new PIDController(1.0, 0.0, 0.0);
        public static PIDController turnPID = new PIDController(1.0, 0.0, 0.0);
        public static PIDController forwardPID = new PIDController(1.0, 0.0, 0.0);
        public static final double epsilonTurn = 1.0;
        public static final double epsilonHorizontal = 0.1;
        public static final double epsilonForward = 0.1;

        public static final double coneEpsilonTurn = 1.0;
        public static final double coneEpsilonHorizontal = 0.1;
        public static final double coneEpsilonForward = 0.1;
        public static final double fieldLength = 15.875508;
    }
    public static final class Ports {
        public static final int intake = 0;
        public static final int leftLeader = 1;
        public static final int leftFollower = 2;
        public static final int rightLeader = 3;
        public static final int rightFollower = 4;
        public static final int gyro = 5;
        public static final int claw = 6;
    }
};