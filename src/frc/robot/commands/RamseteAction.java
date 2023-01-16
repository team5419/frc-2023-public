package frc.robot.commands;
import frc.robot.Constants.ProtoDrive;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.subsystems.Drivetrain;
import java.util.Arrays;

public class RamseteAction extends CommandBase {
    private Drivetrain drivetrain;
    private Pose2d[] poses;
    private boolean reversed;
    DifferentialDriveKinematics driveKinematics;
    SimpleMotorFeedforward feedforward;
    DifferentialDriveVoltageConstraint voltageConstraint;
    DifferentialDriveKinematicsConstraint driveKinematicsConstraint;
    TrajectoryConfig config;
    Trajectory trajectory;
    RamseteController controller;
    DifferentialDriveWheelSpeeds prevSpeed;
    double prevTime;
    Timer timer;
    public RamseteAction(Drivetrain m_drivetrain, Pose2d[] m_poses, boolean m_reversed) {
        drivetrain = m_drivetrain;
        addRequirements(drivetrain);
        poses = m_poses;
        reversed = m_reversed;
        setup();
    }
    public RamseteAction(Drivetrain m_drivetrain, Pose2d[] m_poses) {
        drivetrain = m_drivetrain;
        addRequirements(drivetrain);
        poses = m_poses;
        reversed = false;
        setup();
    }
    private void setup() {

        driveKinematics = new DifferentialDriveKinematics(ProtoDrive.trackWidth);
        feedforward = new SimpleMotorFeedforward(ProtoDrive.ks, ProtoDrive.kv, ProtoDrive.ka);
        voltageConstraint = new DifferentialDriveVoltageConstraint(feedforward, driveKinematics, ProtoDrive.maxVoltage);
        driveKinematicsConstraint = new DifferentialDriveKinematicsConstraint(driveKinematics, ProtoDrive.maxVelocity);

        // again, just trust it
        config = new TrajectoryConfig(ProtoDrive.maxVelocity, ProtoDrive.maxAcceleration);
            config.setKinematics(driveKinematics);
            config.setReversed(!reversed); // our motors are weird
            config.addConstraint(voltageConstraint);
            config.addConstraint(driveKinematicsConstraint);
            config.addConstraint(new CentripetalAccelerationConstraint(
                ProtoDrive.maxCentripetalAcceleration
            ));
            trajectory = TrajectoryGenerator.generateTrajectory(Arrays.asList(poses), config);
            controller = new RamseteController(ProtoDrive.beta, ProtoDrive.zeta);
            prevSpeed = new DifferentialDriveWheelSpeeds(0.0, 0.0);
            prevTime = 0.0;
            timer = new Timer();
    }

    public void initialize() {
        timer.reset();
        timer.start();
        drivetrain.setBrakeMode(true);
    }

    public void execute() {
        double time = timer.get();
        // find the speed that the robot should be at at a specific time in the path
        State sample = trajectory.sample(time);
        Pose2d currentPose = drivetrain.getPose();
        //System.out.println(currentPose.getX());
        ChassisSpeeds chassisSpeed = controller.calculate(currentPose, sample);
        DifferentialDriveWheelSpeeds setSpeed = driveKinematics.toWheelSpeeds(chassisSpeed);
        //System.out.println(setSpeed.leftMetersPerSecond - setSpeed.rightMetersPerSecond);
        // calculate the feedforwards of both sides
        double leftFeedForward = feedforward.calculate(
            setSpeed.leftMetersPerSecond,
            (setSpeed.leftMetersPerSecond - prevSpeed.leftMetersPerSecond) / (time - prevTime)
        );
        double rightFeedForward = feedforward.calculate(
            setSpeed.rightMetersPerSecond,
            (setSpeed.rightMetersPerSecond - prevSpeed.rightMetersPerSecond) / (time - prevTime)
        );
        prevSpeed = setSpeed;
        prevTime = time;

        // set velocity of drivetrain
        drivetrain.setVelocity(
            setSpeed.leftMetersPerSecond,
            setSpeed.rightMetersPerSecond,
            leftFeedForward,
            rightFeedForward
        );
    
        //  System.out.println("m/s: " + setSpeed.rightMetersPerSecond);
        //  System.out.println("m/s: " + prevSpeed.rightMetersPerSecond);
        // System.out.println(drivetrain.getAngle());
        // System.out.println("x: " + drivetrain.getPose().getX());
        // System.out.println("y: " + drivetrain.getPose().getY());
    }

    public void end(boolean interrupted) {
        System.out.println(drivetrain.getAngle());
        drivetrain.setBrakeMode(false);
        drivetrain.drive(0.0, 0.0);
    }

    public boolean isFinished() {
        return timer.get() >= trajectory.getTotalTimeSeconds();
    }
}