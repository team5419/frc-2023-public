package frc.robot;

import frc.robot.modules.ModuleInfo;
import frc.robot.classes.PID;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import java.lang.Math;

public class Constants {
    public static final class Ports {
        public static final int gyro = 0;
    }

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

        public static final ModuleInfo[] info = {
                                                    new ModuleInfo(0, 4, false, true, 8, 0.0), //numbers are placeholders
                                                    new ModuleInfo(1, 5, true, true, 9, 0.0),
                                                    new ModuleInfo(2, 6, false, true, 10, 0.0),
                                                    new ModuleInfo(3, 7, true, true, 11, 0.0)
                                                };

        public static final Translation2d frontLeft = new Translation2d(0.0, 0.0);
        public static final Translation2d frontRight = new Translation2d(0.0, 0.0);
        public static final Translation2d backLeft = new Translation2d(0.0, 0.0);
        public static final Translation2d backRight = new Translation2d(0.0, 0.0);

        public static final Translation2d[] modulePositions = {frontLeft, frontRight, backLeft, backRight};

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeft, frontRight, backLeft, backRight);
    };
};