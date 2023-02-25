package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConerConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.TargetHeights;
import frc.robot.classes.PID;
import frc.robot.subsystems.test.TesterFalcon;
import frc.robot.subsystems.test.TesterMotor;
import frc.robot.subsystems.test.TesterNeo;
import frc.robot.subsystems.test.TesterProFalcon;
import frc.robot.subsystems.test.TesterSubsystem;
import frc.robot.Util;

public class Coner extends TesterSubsystem implements GenericShootIntake {
    private Solenoid soOne;
    //private Solenoid soTwo;
    private double timestamp;
    public Coner(PneumaticHub hub, boolean falcons, boolean velocityControl) {
        super("Cone Shooter", new TesterMotor[] {
            generateTesterMotor("Low motor", falcons, Ports.coneBottom, false),
            generateTesterMotor("High motor", falcons, Ports.coneTop, false)
        }, velocityControl ? (falcons ? ConerConstants.falconVelocities : ConerConstants.neoVelocities) : ConerConstants.percents);

        soOne = hub.makeSolenoid(Ports.conerSolenoidA);
        //soTwo = hub.makeSolenoid(Ports.conerSolenoidB);
        soOne.set(false);
        //soTwo.set(false);
        timestamp = -1.0;
    }
    public static TesterMotor generateTesterMotor(String name, boolean falcons, int id, boolean pro) {
        if(falcons) {
            if(pro) {
                com.ctre.phoenixpro.hardware.TalonFX fx = new com.ctre.phoenixpro.hardware.TalonFX(id, "canivore");
                TalonFXConfiguration config = Util.getSetup(false, new PID(0.5, 0, 0), true, 1.0);
                config.CurrentLimits.SupplyCurrentLimit = 20;
                return new TesterProFalcon(name, fx, config);
            }
            TalonFX motor = new TalonFX(id, "canivore");
            Util.setUpMotor(motor, new PID(0.1, 0, 0), true, 1.0);
            SupplyCurrentLimitConfiguration current = new SupplyCurrentLimitConfiguration();
            current.currentLimit = 20.0;
            motor.configSupplyCurrentLimit(current);
            return new TesterFalcon(name, motor);
        }
        return new TesterNeo(name, Util.setUpMotor(new CANSparkMax(id, MotorType.kBrushless), false, true));
    }
    public void shoot(String height) {
        run(height);
        timestamp = -1.0;
    }
    public void stop(String height) {
        super.stop();
            soOne.set(false);
        
    }
    public void setup(String height) {
        soOne.set(true);
        if(height != TargetHeights.INTAKE) {
            run(TargetHeights.INTAKE);
            if(timestamp == -1.0) {
                timestamp = Timer.getFPGATimestamp();
            }
        }
    }
    public boolean donePrepping(String height) {
        return timestamp >= 0.0 && Timer.getFPGATimestamp() - timestamp >= 0.625 && soOne.get();
    }
    public SubsystemBase subsystem() {return this;}
    public final double getAngle() {
        return 180.0;
    }
    public final double getOffset() {
        return 0.0;
    }
    public final boolean prepsByDefault() {
        return false;
    }
    public final double getDistance(String height) {
        return 2.1; // a little off so that we can rotate freely
    }
    public final double getLimelightDistance(String height) {
        return 0.3205; // 0.569
    }
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
    public void inOut(boolean in) {
        motors[0].run(ConerConstants.inOutVelocity * (in ? -1 : 1));
        motors[1].run(ConerConstants.inOutVelocity * (in ? 1 : -1));
    }
}