package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.Intake;

// import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

public class ConeIntake extends SubsystemBase {
    private CANSparkMax motor;
    public boolean outTake;

    public ConeIntake(boolean _outTake){
        this.outTake = _outTake;
        motor = new CANSparkMax(Ports.cone, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kCoast);
        motor.setInverted(outTake);
        //setSensorPhase(false)
        motor.setSmartCurrentLimit(40);
        motor.setClosedLoopRampRate(1.0);
        motor.setControlFramePeriodMs(50);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);

    }
    public void intake(){
        motor.set(Intake.outputPercentage);
    }
    public void outtake(){
        motor.set(Intake.reversePercentage);
    }

    public void periodic() {
        //yButton.toggleOnTrue(() -> drive(0.9, 0.0));
        
    }
    public void simulationPeriodic() {

    }
}
//1 Motor that can spin two directions based on different button inouts
//incoder on the shaft that changes at what angle the "arm"