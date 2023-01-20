package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;


public class Intake extends SubsystemBase {
    public TalonSRX /*CANSparkMax */ motor;
    private TalonSRX /*CANSparkMax */ motor2;
    private boolean running;
    public Intake(ShuffleboardTab _tab) {
        motor = new TalonSRX(17);//new CANSparkMax(Ports.intake, MotorType.kBrushless);
        motor2 = new TalonSRX(10);
        //Util.setUpMotor(motor, true, false);

        running = false;

        motor.configFactoryDefault(100);
        motor.setInverted(false);
        motor.configPeakOutputForward(1.0);
        motor.configPeakOutputReverse(-1.0);
        motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40.0, 0.0, 0.0), 100);
        motor.config_kP(0, 1.0);
        motor.config_kI(0, 0.0);
        motor.config_kD(0, 0.0);
        motor.config_kF(0, 0.0);
        motor2.configFactoryDefault(100);
        motor2.setInverted(false);
        motor2.configPeakOutputForward(1.0);
        motor2.configPeakOutputReverse(-1.0);
        motor2.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40.0, 0.0, 0.0), 100);


        _tab.addNumber("Actual Velocity", () -> flyWheelVelocity(motor));
        _tab.addBoolean("Running?", () -> running);
    }
    public void run(double percent) {
        motor.set(ControlMode.PercentOutput, percent);
        motor2.set(ControlMode.PercentOutput, -percent);
        running = true;
    }
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
    public double flyWheelVelocity(TalonSRX _motor) {
        return _motor.getSelectedSensorVelocity(0);
    }
}