package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants.Ports;

<<<<<<< HEAD
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
=======
public class IntakeDeploy extends TwoPhaseSubsystem { // intake deploy system to raise/lower the intake
    private Compressor compressor; // compressor that fills with air over time so that the solenoid can be activated
    private Solenoid solenoid; // turn this on and off to raise or lower the intake
    public IntakeDeploy() {
        super(); // instantiate the twophasesubsystem because the intake deploy is always either on or off
        compressor = new Compressor(0, PneumaticsModuleType.CTREPCM); // instantiate the compressor on port 0
        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0); // instantiate the solenoid on port 0 as well
        solenoid.set(false); // turn off the solenoid for now
        compressor.enableDigital(); // turn on the compressor so it builds up pressure over time
    }
    public void _run() { // turn the solenoid on or off depending on the current state
        solenoid.set(on);
>>>>>>> e1671c2ed58933bd695d6d35ddbaf33607ca688c
    }
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
}