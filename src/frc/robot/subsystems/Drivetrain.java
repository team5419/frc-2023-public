package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.*;


public class Drivetrain extends SubsystemBase {
    TalonFX motor1 = new TalonFX(1);
    TalonFX motor2 = new TalonFX(2);
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
    public void drive(double throttle, double rotation){
        motor1.set(ControlMode.PercentOutput, throttle + rotation);
        motor2.set(ControlMode.PercentOutput, throttle - rotation);
    }
    
}