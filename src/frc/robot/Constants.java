package frc.robot;
import frc.robot.classes.PID;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
    };
    public static final class Ports {
        public static final int intake = 0;
    }
};