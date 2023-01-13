package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;

public class CubeShooter extends SubsystemBase {
    private TalonFX motor;
    
    public CubeShooter(){
        motor = new TalonFX(1);
    }
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
   
    public void shoot(double power){
        motor.set(TalonFXControlMode.PercentOutput,power);
    }
}