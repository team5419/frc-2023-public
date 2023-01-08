package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
public class Drivetrain extends SubsystemBase {

    TalonFX motor4;
    TalonFX motor1;
    TalonFX motor2;
    TalonFX motor3;
    public Drivetrain() {
        motor1= new TalonFX(1);
        motor2= new TalonFX(2);
        motor3= new TalonFX(3);
        motor4= new TalonFX(4);
        
    }
    public void periodic() {

    }
    public void simulationPeriodic() {


    }
    public void drive(double speed){ 
        motor4.set(TalonFXControlMode.PercentOutput, speed); // runs the motor at 50% power
        motor3.set(TalonFXControlMode.PercentOutput, speed); // runs the motor at 50% power
        motor2.set(TalonFXControlMode.PercentOutput, speed); // runs the motor at 50% power
        motor1.set(TalonFXControlMode.PercentOutput, speed); // runs the motor at 50% power
    }
    public void turnleft(double speed){
        motor2.set(TalonFXControlMode.PercentOutput, speed); // runs the motor at 50% power
        motor1.set(TalonFXControlMode.PercentOutput, speed); // runs the motor at 50% power

    }

}