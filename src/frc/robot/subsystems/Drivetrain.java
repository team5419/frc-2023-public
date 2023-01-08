package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    private static TalonFX motor1 = new TalonFX(0);
    private static TalonFX motor2 = new TalonFX(1);
    private static TalonFX motor3 = new TalonFX(2);
    private static TalonFX motor4 = new TalonFX(3);
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
    public static void drive(double y, double x){

        motor1.set(TalonFXControlMode.PercentOutput, y-x); 
        motor2.set(TalonFXControlMode.PercentOutput, y-x); 
        motor3.set(TalonFXControlMode.PercentOutput, y+x); 
        motor4.set(TalonFXControlMode.PercentOutput, y+x); 
    }
}