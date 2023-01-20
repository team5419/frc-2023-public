package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.hal.simulation.CTREPCMDataJNI;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

import edu.wpi.first.wpilibj.Compressor;

// import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class Suction extends SubsystemBase {
    private Compressor pcmCompressor;
    private Solenoid solenoid;

    private boolean airFlow;

    public Suction() {
        airFlow = false;

        pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
        solenoid.set(airFlow);

        pcmCompressor.enableDigital();
    }

    public void airIn() {
        airFlow = !airFlow;
        solenoid.set(airFlow);
    }
}