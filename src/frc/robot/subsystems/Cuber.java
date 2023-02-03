package frc.robot.subsystems;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.CubeShooter;
import frc.robot.Constants.Ports;
import frc.robot.subsystems.test.TesterMotor;
import frc.robot.subsystems.test.TesterNeo;
import frc.robot.subsystems.test.TesterSetting;
import frc.robot.subsystems.test.TesterSubsystem;
import frc.robot.Util;

public class Cuber extends TesterSubsystem implements GenericShootIntake {
    public Cuber() {
        super("Cube Shooter", new TesterMotor[] {
            new TesterNeo("Main", Util.setUpMotor(
                new CANSparkMax(Ports.intake, MotorType.kBrushless), false, false
            ))
        }, Map.of("low", new TesterSetting(new double[] { CubeShooter.outtakeSpeedLow }),
                  "mid", new TesterSetting(new double[] { CubeShooter.outtakeSpeedMid }),
                  "high", new TesterSetting(new double[] { CubeShooter.outtakeSpeedHigh }),
                  "intake", new TesterSetting(new double[] { CubeShooter.intakeSpeed })));
    }
    public void shootHigh() { // set the motor to a given percent output
        run("high");
    }
    public void shootMid() {
        run("mid");
    }
    public void shootLow() {
        run("low");
    }
    public void intake() {
        run("intake");
    }
    public void stop() {
        super.stop();
    }
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
}