package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.Ports;
import frc.robot.Util;
import frc.robot.Constants.Arm;

public class EverybotArm extends SubsystemBase {
    private CANSparkMax intakeMotor;
    private TalonFX leftArm;
    private TalonFX rightArm;
    // private GenericEntry widget;
    public EverybotArm() {
        intakeMotor = new CANSparkMax(Ports.everyIntakeMotor, MotorType.kBrushless);
        Util.setUpMotor(intakeMotor, false, false);
        leftArm = new TalonFX(Ports.everyArm0);
        Util.setUpMotor(leftArm, Arm.PID, true, 1.0);
        rightArm = new TalonFX(Ports.everyArm1);
        Util.setUpMotor(rightArm, Arm.PID, true, 1.0);
        rightArm.follow(leftArm); // hopefully this works
    }

    public void gotoPosition(double ticks) {
        leftArm.set(ControlMode.Position, ticks);
    }

    public void start(boolean reverse) {
        intakeMotor.set(reverse ? Arm.outtakeSpeed : Arm.intakeSpeed);
    }

    public void stop() {
        intakeMotor.set(0.0);
    }

    public void periodic() {

    }

    public void simulationPeriodic() {

    }
}