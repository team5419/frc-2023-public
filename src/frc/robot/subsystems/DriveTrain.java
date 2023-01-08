package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.fasterxml.jackson.annotation.JacksonInject.Value;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

    TalonFX motor0 = new TalonFX(0);
    TalonFX motor1 = new TalonFX(1);
    TalonFX motor2 = new TalonFX(2);
    TalonFX motor3 = new TalonFX(3);

    public void periodic() {

    }
    public void simulationPeriodic() {
        
    }
    public void drive(Double throttle, double turnAmount){
        motor0.set(TalonFXControlMode.PercentOutput, throttle - turnAmount);
        motor1.set(TalonFXControlMode.PercentOutput, throttle - turnAmount);
        motor2.set(TalonFXControlMode.PercentOutput, throttle + turnAmount);
        motor3.set(TalonFXControlMode.PercentOutput, throttle + turnAmount);
    }

}