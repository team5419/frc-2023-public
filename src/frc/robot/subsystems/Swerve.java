package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.*;
// import frc.robot.subsystems.Vision;
import frc.robot.commands.ResetGyro;
// import frc.robot.DriveConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.modules.SwerveModule;
import frc.robot.modules.ISwerveModule;
// import frc.robot.modules.SimulatedSwerveModule;
// import frc.robot.modules.Module;
import frc.robot.Util;
// import frc.robot.Ports;
import frc.robot.Constants.*;
import java.lang.Math.*;

public class Swerve extends SubsystemBase {
    public ISwerveModule[] drivers = {
                                        new SwerveModule(Drive.info[0], 0),
                                        new SwerveModule(Drive.info[1], 1),
                                        new SwerveModule(Drive.info[2], 2),
                                        new SwerveModule(Drive.info[3], 3)
                                     };
    public Pigeon2 gyro;
    private double yaw;
    private double inverted;
    private Field2d field;
    private ChassisSpeeds previousMove;
    public boolean slowMode;
    private double angle;
    public boolean brakeMode;
    private SwerveDriveOdometry odometry = new SwerveDriveOdometry(Drive.kinematics, Rotation2d.fromDegrees(angle), new SwerveModulePosition[] {
                                                                                                                                                    drivers[0].getPosition(),
                                                                                                                                                    drivers[1].getPosition(),
                                                                                                                                                    drivers[2].getPosition(),
                                                                                                                                                    drivers[3].getPosition()
                                                                                                                                               });

    public Swerve() {
        gyro  = new Pigeon2(Ports.gyro);
        yaw = 0.0;
        inverted = 1.0;
        field = new Field2d();
        previousMove = new ChassisSpeeds();
        slowMode = false;
        angle = gyro.getYaw();
        brakeMode = false;

        for (int i = 0; i < drivers.length; i++) {
            drivers[i].setBrakeMode(brakeMode);
        }

        gyro.configFactoryDefault(100);
        gyro.setYaw(0.0, 100);

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        ShuffleboardLayout layout = tab.getLayout("Main", BuiltInLayouts.kList).withPosition(0, 0).withSize(1, 5);
        SmartDashboard.putData("Field", field);
        layout.addNumber("gyro", () -> {return this.gyro.getYaw();});
        layout.add("reset gyro", new ResetGyro(this));
        layout.addBoolean("slowmode", () -> {return this.slowMode;});
        layout.addNumber("x position", () -> {return pose().getX();});
        layout.addNumber("y position", () -> {return pose().getY();});
    }

    public Pose2d pose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(Rotation2d.fromDegrees(angle), new SwerveModulePosition[] {
                                                                                            drivers[0].getPosition(),
                                                                                            drivers[1].getPosition(),
                                                                                            drivers[2].getPosition(),
                                                                                            drivers[3].getPosition()
                                                                                         }, pose);
    }

    public void resetOdometry() {
        odometry.resetPosition(Rotation2d.fromDegrees(angle), new SwerveModulePosition[] {
                                                                                            drivers[0].getPosition(),
                                                                                            drivers[1].getPosition(),
                                                                                            drivers[2].getPosition(),
                                                                                            drivers[3].getPosition()
                                                                                         }, new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
    }

    public void stop() {
        this.drive(0.0, 0.0, 0.0);
    }

    public void brake() {
        drivers[0].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)), false, false, true);
        drivers[1].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(315.0)), false, false, true);
        drivers[2].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0)), false, false, true);
        drivers[3].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(225.0)), false, false, true);
    }

    public void drive(double forward, double left, double rotation, boolean fieldCentric, boolean pid) {
        ChassisSpeeds speeds = (fieldCentric) ? ChassisSpeeds.fromFieldRelativeSpeeds(forward, left, rotation, Rotation2d.fromDegrees(this.angle)) : new ChassisSpeeds(forward, left, rotation);
        this.previousMove = speeds;
        SwerveModuleState[] states = Drive.kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Drive.SwerveRamsete.maxVelocity);
        updateMotors(states, pid, forward == 0.0 && left == 0.0 && rotation == 0.0);
    }

    public void drive(double forward, double left, double rotation, boolean pid) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, left, rotation, Rotation2d.fromDegrees(this.angle));
        this.previousMove = speeds;
        SwerveModuleState[] states = Drive.kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Drive.SwerveRamsete.maxVelocity);
        updateMotors(states, pid, forward == 0.0 && left == 0.0 && rotation == 0.0);
    }

    public void drive(double forward, double left, double rotation) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, left, rotation, Rotation2d.fromDegrees(this.angle));
        this.previousMove = speeds;
        SwerveModuleState[] states = Drive.kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Drive.SwerveRamsete.maxVelocity);
        updateMotors(states, false, forward == 0.0 && left == 0.0 && rotation == 0.0);
    }

    public void updateMotors(SwerveModuleState[] myStates, boolean pid, boolean preventTurn) {
        for(int i = 0; i < drivers.length; i++) {
            drivers[i].setDesiredState(myStates[i], preventTurn, this.slowMode, pid);
        }
    }

    public double withDeadband(double movement, double deadband) {
        if(Math.abs(movement) <= deadband) return 0.0;
        if(Math.abs(movement) > 1.0) return Math.signum(movement);
        return movement;
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] moduleState = new SwerveModuleState[drivers.length];
        for(int i = 0; i < moduleState.length; i++) {
            moduleState[i] = drivers[i].getState();
        }
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[drivers.length];
        for(int i = 0; i < modulePositions.length; i++) {
            modulePositions[i] = drivers[i].getPosition();
        }
    }

    public double getAverageSpeed() {
        double total = 0.0;
        for(int i = 0; i < drivers.length; i++) {
            total += Util.nativeUnitsToMetersPerSecond(drivers[i].getDrive());
        };
        return total / drivers.length;
    }

    public void periodic() {
        SwerveModulePosition[] res = getPositions();
        Rotation2d cangle = Rotation2d.fromDegrees(angle);
        Pose2d cpose = pose();
        odometry.update(cangle, res);

        for(int i = 0; i < drivers.length; i++) {
            Translation2d modulePositionFromChassis = Drive.modulePositions[i]
                                                                              .rotateBy(cangle)
                                                                              .plus(cpose.getTranslation());
            Pose2d pose = new Pose2d(modulePositionFromChassis, drivers[i].getTurn().plus(cpose.getRotation()));
            field.getObject("Swerve Module ${i}").setPose(pose);
        }
        field.setRobotPose(cpose);

        SmartDashboard.putNumber("angle (deg)", angle);
        SmartDashboard.putNumber("x m", pose().getX());
        SmartDashboard.putNumber("y m", pose().getY());
        SmartDashboard.putBoolean("brake", brakeMode);
        SmartDashboard.putNumber("forward m/s", previousMove.vxMetersPerSecond);
        SmartDashboard.putNumber("sideways m/s", previousMove.vyMetersPerSecond);
        SmartDashboard.putNumber("turning (rad)", previousMove.omegaRadiansPerSecond);
    }

    public void simulationPeriodic() {
        
    }
}