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
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;

public class Drivetrain extends SubsystemBase {
    TalonFX leftLeader;
    TalonFX leftFollower;
    TalonFX rightLeader;
    TalonFX rightFollower;
    PigeonIMU gyro;
    DifferentialDriveOdometry odometry;

    public Drivetrain() {
        leftLeader = new TalonFX(Ports.leftLeader);
        leftFollower = new TalonFX(Ports.leftFollower);
        rightLeader = new TalonFX(Ports.rightLeader);
        rightFollower = new TalonFX(Ports.rightFollower);
        gyro = new PigeonIMU(Ports.gyro);

        Util.setUpMotor(leftLeader);
        Util.setUpMotor(leftFollower);
        Util.setUpMotor(rightLeader);
        Util.setUpMotor(rightFollower);
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        // constructs object with angle from gyro (assuming starting position is (0,0))
        odometry = new DifferentialDriveOdometry(new Rotation2d(getAngle()), getLeftDistance(), getRightDistance());
    }

    
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void periodic() {
        // update the odometry of the field with new gyro and encoder values
        odometry.update(Rotation2d.fromDegrees(getAngle()), getLeftDistance(), getRightDistance());
    }
    public void simulationPeriodic() {
        
    }
    public void drive(double throttle, double turnAmount){
        leftLeader.set(TalonFXControlMode.PercentOutput, throttle - turnAmount);
        rightLeader.set(TalonFXControlMode.PercentOutput, throttle + turnAmount);
    }
    public void setVelocity(double leftVelocity, double rightVelocity, double leftFF, double rightFF) {
        leftLeader.set(
            ControlMode.Velocity, Util.metersPerSecondToNativeUnits(leftVelocity),
            DemandType.ArbitraryFeedForward, leftFF / 12.0
        );
        rightLeader.set(
            ControlMode.Velocity, Util.metersPerSecondToNativeUnits(rightVelocity),
            DemandType.ArbitraryFeedForward, rightFF / 12.0
        );
    }

    public double getAngle() {
        return -gyro.getFusedHeading();
    }

    public double getLeftDistance() {
        return Util.nativeUnitsToMetersPerSecond(leftLeader.getSelectedSensorPosition(0));
    }
    public double getRightDistance() {
        return Util.nativeUnitsToMetersPerSecond(rightLeader.getSelectedSensorPosition(0));
    }

    public void setBrakeMode(boolean brakeMode) {
        leftLeader.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
        leftFollower.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
        rightLeader.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
        rightFollower.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
    }
}