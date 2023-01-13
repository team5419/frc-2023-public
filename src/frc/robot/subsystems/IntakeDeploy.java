package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class IntakeDeploy extends TwoPhaseSubsystem {
    private Compressor compressor;
    private Solenoid solenoid;
    public IntakeDeploy() {
        super();
        compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        solenoid.set(false);
        compressor.enableDigital();
    }
    public void _run() {
        solenoid.set(on);
    }
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
}