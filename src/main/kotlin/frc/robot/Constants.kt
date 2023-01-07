package frc.robot;
import frc.robot.classes.PID;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import kotlin.math.PI
object Constants {
    object Drive {
        val DrivePID: PID = PID(0.1, 0.0, 0.0);
        val TurnPID: PID = PID(8.0, 0.0, 0.0);
        const val slow: Double = 0.2;
        const val maxVelocity: Double = 4.0;
        const val kv: Double = (2.1737 / 12);
        const val ka: Double = (0.29281 / 12);
        const val ks: Double = (0.63566 / 12);
        val feedForward: SimpleMotorFeedforward = SimpleMotorFeedforward(ks, kv, ka);
        val turnController: PIDController = PIDController(TurnPID.p, TurnPID.i, TurnPID.d);

        const val driveGearRatio: Double = 6.75;
        const val driveCPR: Double = 2048.0;
        const val ticksPerRotation: Double = (driveCPR * driveGearRatio);
        const val wheelRadius = 0.0508; // m
        const val wheelDiameter = wheelRadius * 2.0; // m
        const val wheelCircumference = wheelDiameter * PI; // m
    };
};