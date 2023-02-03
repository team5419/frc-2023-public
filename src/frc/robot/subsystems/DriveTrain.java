package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;

public class Drivetrain extends SubsystemBase { // differential (non-swerve) drivetrain
    private TalonFX leftLeader; // the front left motor
    private TalonFX leftFollower; // the back left motor
    private TalonFX rightLeader; // the front right motor
    private TalonFX rightFollower; // the back right motor
    private DifferentialDriveOdometry odometry; // this is used to keep track of the robot's position
    public Pigeon2 gyro; // a gyrometer that's used to read our current angle
    private double mod;

    public Drivetrain() {
        leftLeader = new TalonFX(Ports.leftLeader); // instantiate all four motors using predefined ports
        leftFollower = new TalonFX(Ports.leftFollower);
        rightLeader = new TalonFX(Ports.rightLeader);
        rightFollower = new TalonFX(Ports.rightFollower);
        gyro = new Pigeon2(Ports.gyro); // initialize the gyrometer at a given port
        gyro.configFactoryDefault(100); // configure the gyro by resetting its current angle
        gyro.setYaw(0.0, 100);

        Util.setUpMotor(leftLeader); // set up all four motors
        Util.setUpMotor(leftFollower);
        Util.setUpMotor(rightLeader, false, true); // the right motors happen to be inverted
        Util.setUpMotor(rightFollower, false, true);
        leftFollower.follow(leftLeader); // tell the followers to follow the leaders so that they always copy the behavior of the leader
        rightFollower.follow(rightLeader);

        // constructs object with angle from gyro (assuming starting position is (0,0))
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()), getLeftDistance(), getRightDistance());
    }

    
    public Pose2d getPose() { // estimate the current position of the robot using the odometry
        return odometry.getPoseMeters();
    }

    public void periodic() { // update the odometry using the current robot angle and the total distance values from both leader motors
        odometry.update(Rotation2d.fromDegrees(getAngle()), getLeftDistance(), getRightDistance());
    }
    public void simulationPeriodic() {
        
    }
    public void drive(double throttle, double turnAmount, boolean slow){ // drive the robot with a given throttle amount and turn amount
        if (slow){
            mod = 0.5;
        } else {
            mod = 1;
        }

        double turnSense = 0.75;

        if (throttle <= 0.02){
            leftLeader.set(TalonFXControlMode.PercentOutput, ( -turnAmount * mod); // the left motors' outputs are decreased by the turn amount
            rightLeader.set(TalonFXControlMode.PercentOutput, (turnAmount * mod);
        } else {
            leftLeader.set(TalonFXControlMode.PercentOutput, (throttle - (turnAmount * turnSense)) * mod); // the left motors' outputs are decreased by the turn amount
            rightLeader.set(TalonFXControlMode.PercentOutput, (throttle + (turnAmount * turnSense)) * mod); // the right motors' outputs are increased by the turn amount
            // picture this in your head - this way, the left and right sides move in opposite direction to produce a turn
        }
    }
    public void setVelocity(double leftVelocity, double rightVelocity, double leftFF, double rightFF) {
        leftLeader.set( // set the left and right sides using velocity control and feed forward to get more accurate control during auto
            ControlMode.Velocity, Util.metersPerSecondToNativeUnitsProto(leftVelocity),
            DemandType.ArbitraryFeedForward, leftFF / 12.0
        );
        rightLeader.set(
            ControlMode.Velocity, Util.metersPerSecondToNativeUnitsProto(rightVelocity),
            DemandType.ArbitraryFeedForward, rightFF / 12.0
        );
    }

    public double getAngle() {
        return -gyro.getYaw(); // grab the robot's current angle based on the gyro
    }

    public double getLeftDistance() { // get the total distance, in meters, that the left side has traveled
        return Util.nativeUnitsToMetersProto(leftLeader.getSelectedSensorPosition(0));
    }
    public double getRightDistance() { // get the total distance, in meters, that the right side has traveled
        return Util.nativeUnitsToMetersProto(rightLeader.getSelectedSensorPosition(0));
    }

    public void setBrakeMode(boolean brakeMode) { // enable or disable brake mode for all four motors to change whether they coast when stopped
        leftLeader.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
        leftFollower.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
        rightLeader.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
        rightFollower.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
    }
}