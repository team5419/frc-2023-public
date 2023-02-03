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
    private TalonFX leftArm;
    private TalonFX rightArm;
    public EverybotArm() {
        leftArm = new TalonFX(Ports.everyArm0);
        Util.setUpMotor(leftArm, Arm.PID, true, 1.0);
        rightArm = new TalonFX(Ports.everyArm1);
        Util.setUpMotor(rightArm, Arm.PID, true, 1.0);
        rightArm.follow(leftArm); // hopefully this works
    }

    public void gotoPosition(double ticks) {
        leftArm.set(ControlMode.Position, ticks);
    }

    public void periodic() {

    }

    public void simulationPeriodic() {

    }
}