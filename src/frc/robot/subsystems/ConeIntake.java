package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConeIntake extends SubsystemBase {
    motor = new TalonFX(Ports.claw);
    Util.setUpMotor(motor);
    public void periodic() {
        yButton.toggleOnTrue(() -> drive(0.9, 0.0));
        
    }
    public void simulationPeriodic() {

    }
}
//1 Motor that can spin two directions based on different button inouts
//incoder on the shaft that changes at what angle the "arm"