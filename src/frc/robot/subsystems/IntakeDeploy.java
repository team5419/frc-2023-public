// package frc.robot.subsystems;
// import edu.wpi.first.wpilibj.Compressor;
// import edu.wpi.first.wpilibj.PneumaticHub;
// import edu.wpi.first.wpilibj.Solenoid;
// import frc.robot.Constants.Ports;

// public class IntakeDeploy extends TwoPhaseSubsystem {
//     private Compressor compressor;
//     //private DoubleSolenoid solenoid;
//     private Solenoid soOne;
//     private Solenoid soTwo;
//     private PneumaticHub hub;
//     public IntakeDeploy() {
//         super();
//         this.hub = new PneumaticHub();
//         compressor = this.hub.makeCompressor();
//         //solenoid = hub.makeDoubleSolenoid(Ports.solenoidA, Ports.solenoidB);
//         //compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
//         //solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Ports.solenoidA, Ports.solenoidB);
//         //solenoid.set(DoubleSolenoid.Value.kOff);
//         soOne = this.hub.makeSolenoid(Ports.solenoidA);
//         soTwo = this.hub.makeSolenoid(Ports.solenoidB);
//         soOne.set(false);
//         soTwo.set(false);
//         compressor.enableDigital();
//     }
//     public void _run() {
//         //System.out.println(on);
//         soOne.set(on);
//         soTwo.set(!on);
//         //solenoid.set(on ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kOff);
//     }
//     public void periodic() {

//     }
//     public void simulationPeriodic() {

//     }
// }