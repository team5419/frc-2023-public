package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.Arm;
import frc.robot.Constants.CubeShooter;
import frc.robot.Constants.EverybotConeConstants;
import frc.robot.Constants.Ports;
import frc.robot.subsystems.test.TesterFalcon;
import frc.robot.subsystems.test.TesterMotor;
import frc.robot.subsystems.test.TesterNeo;
import frc.robot.subsystems.test.TesterSetting;
import frc.robot.subsystems.test.TesterSubsystem;
import frc.robot.Util;

public class EverybotConer extends TesterSubsystem implements GenericShootIntake {
    private EverybotArm arm;
    public EverybotConer(EverybotArm arm) {
        super("Everybot Cone Shooter", new TesterMotor[] {
            new TesterFalcon("Main", Util.setUpMotor(
                new TalonFX(Ports.everyIntakeMotor), false, false
            ))
        }, Map.of("low", new TesterSetting(new double[] { EverybotConeConstants.outtakeSpeedHigh }),
                  "mid", new TesterSetting(new double[] { EverybotConeConstants.outtakeSpeedMid }),
                  "high", new TesterSetting(new double[] { EverybotConeConstants.outtakeSpeedHigh }),
                  "intake", new TesterSetting(new double[] { EverybotConeConstants.intakeSpeed })));
        this.arm = arm;
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
    public void setup(int height) {
        arm.gotoPosition(Arm.outPosition);
    }
    public void stop() {
        super.stop();
        arm.gotoPosition(Arm.inPosition);
    }
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
}