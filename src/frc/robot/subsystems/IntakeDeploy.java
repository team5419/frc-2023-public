package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants.Ports;

public class IntakeDeploy extends TwoPhaseSubsystem {
    private Compressor compressor;
    private DoubleSolenoid solenoid;
    private PneumaticHub hub;
    public IntakeDeploy() {
        super();
        hub = new PneumaticHub();
        compressor = hub.makeCompressor();
        solenoid = hub.makeDoubleSolenoid(Ports.solenoidA, Ports.solenoidB);
        //compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
        //solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Ports.solenoidA, Ports.solenoidB);
        solenoid.set(DoubleSolenoid.Value.kOff);
        compressor.enableDigital();
    }
    public void _run() {
        System.out.println(on);
        solenoid.set(on ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kOff);
    }
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
}