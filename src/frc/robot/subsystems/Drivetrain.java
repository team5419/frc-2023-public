package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.*;
// import frc.robot.subsystems.Vision;
// import frc.robot.commands.ResetGyro;
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

public class Drivetrain extends SubsystemBase {
    public ISwerveModule[] drivers = new ISwerveModule[4];
    for (int i = 0; i < Drive.info.length; i++) {
        drivers[i] = new SwerveModule(Drive.info[i], i);
    }
    public Pigeon2 gyro = new Pigeon2(Ports.gyro);
    private double yaw = 0.0;
    private double inverted = 1.0;
    private Field2d field = new Field2d();
    private ChassisSpeeds previousMove = new ChassisSpeeds();
    public boolean slowMode = false;

    public Drivetrain() {
        gyro.configFactoryDefault(100);
        gyro.setYaw(0.0, 100);

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        ShuffleboardLayout layout = tab.getLayout("Main", BuiltInLayouts.kList).withPosition(0, 0).withSize(1, 5);
        SmartDashboard.putData("Field", field);
        layout.addNumber("gyro", angle)
        layout.add("reset gyro", ResetGyro(this));
        layout.addBoolean("slowmode", this.slowMode);
        layout.addNumber("x position", pose().getX());
        layout.addNumber("y position", pose().getY());
    }

    public void periodic() {

    }
    public void simulationPeriodic() {

    }
}