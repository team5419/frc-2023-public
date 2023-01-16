package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class Intake extends SubsystemBase {
    private TalonFX /*CANSparkMax */ motor;
    public Intake(ShuffleboardTab _tab) {
        motor = new TalonFX(32);//new CANSparkMax(Ports.intake, MotorType.kBrushless);
        //Util.setUpMotor(motor, true, false);

        motor.configFactoryDefault(100);
        motor.setInverted(true);
        motor.configPeakOutputForward(1.0);
        motor.configPeakOutputReverse(-1.0);

        _tab.addNumber("Actual Velocity", () -> flyWheelVelocity(motor));
    }
    public void run(double percent) {
        motor.set(ControlMode.PercentOutput, percent);
    }
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
    public double flyWheelVelocity(TalonFX _motor) {
        return _motor.getSelectedSensorVelocity(0);
    }


}