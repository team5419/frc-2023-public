package frc.robot.subsystems;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.Ports;
import frc.robot.Util;

public class EverybotArm extends SubsystemBase {
    private CANSparkMax intakeMotor;
    private CANSparkMax[] armMotors;
    // private GenericEntry widget;
    public EverybotArm() {
        intakeMotor = new CANSparkMax(Ports.everyIntakeMotor, MotorType.kBrushless);
        armMotors[0] = new CANSparkMax(Ports.everyArm0, MotorType.kBrushless);
        armMotors[1] = new CANSparkMax(Ports.everyArm1, MotorType.kBrushless);
        Util.setUpMotor(intakeMotor, false, false);
        for (int i = 0; i < armMotors.length; i++) {
            Util.setUpMotor(armMotors[i], false, i % 2 == 0 ? false : true);
        }
    }

    public void arm(double speed) {
        for (int i = 0; i < armMotors.length; i++) {
            armMotors[i].set(speed);
        }
    }

    public void start(boolean reverse) {
        intakeMotor.set(reverse ? -1.0 : 1.0);
    }

    public void stop() {
        intakeMotor.set(0.0);
    }

    public void periodic() {

    }

    public void simulationPeriodic() {

    }
}