package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConerConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.TargetHeights;
import frc.robot.classes.PID;
import frc.robot.subsystems.test.TesterFalcon;
import frc.robot.subsystems.test.TesterMotor;
import frc.robot.subsystems.test.TesterNeo;
import frc.robot.subsystems.test.TesterProFalcon;
import frc.robot.subsystems.test.TesterSetting;
import frc.robot.subsystems.test.TesterSubsystem;
import frc.robot.Util;

public class Coner extends TesterSubsystem implements GenericShootIntake {
    private Solenoid soOne;
    private Solenoid soTwo;
    private double timestamp;
    private GenericEntry dist;
    private final double defaultDist = 0.4962;
    public Coner(boolean falcons, boolean velocityControl) {
        super("Cone Shooter", new TesterMotor[] {
            generateTesterMotor("Low motor", falcons, Ports.coneBottom, false, false),
            generateTesterMotor("High motor", falcons, Ports.coneTop, false, true)
        }, velocityControl ? (falcons ? falconVelocities : neoVelocities) : percents);

        soOne = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.conerSolenoidA); //hub.makeSolenoid(Ports.conerSolenoidA);
        soTwo = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.conerSolenoidB);//hub.makeSolenoid(Ports.conerSolenoidB);
        soOne.set(false);
        soTwo.set(true);
        timestamp = -1.0;
        ShuffleboardTab tab = Shuffleboard.getTab("Master");
        dist = tab.add("Limelight dist", defaultDist)
            .withPosition(0, 3)
            .withSize(1, 1)
            .getEntry();
    }
    public static TesterMotor generateTesterMotor(String name, boolean falcons, int id, boolean pro, boolean inverted) {
        if(falcons) {
            if(pro) {
                com.ctre.phoenixpro.hardware.TalonFX fx = new com.ctre.phoenixpro.hardware.TalonFX(id, "canivore");
                TalonFXConfiguration config = Util.getSetup(inverted, new PID(0.5, 0, 0), true, 1.0);
                config.CurrentLimits.SupplyCurrentLimit = 20;
                return new TesterProFalcon(name, fx, config);
            }
            TalonFX motor = new TalonFX(id, "canivore");
            Util.setUpMotor(motor, false, inverted, new PID(0.1, 0, 0), true, 1.0);
            
            SupplyCurrentLimitConfiguration current = new SupplyCurrentLimitConfiguration();
            current.currentLimit = 20.0;
            motor.configSupplyCurrentLimit(current);
            return new TesterFalcon(name, motor);
        }
        return new TesterNeo(name, Util.setUpMotor(new CANSparkMax(id, MotorType.kBrushless), inverted, true));
    }
    public void shoot(String height) {
        if(height == TargetHeights.LOW) {
            if(!soOne.get()) {
                soOne.set(true);
            }
            if(soTwo.get()) {
                soTwo.set(false);
            }
        }
        if(height == TargetHeights.FAR) {
            run(TargetHeights.HIGH);
        } else {
            run(height);
        }
        timestamp = -1.0;
    }
    public void stop(String height) {
        timestamp = -1.0;
        super.stop();
            soOne.set(false);
            soTwo.set(true);
        
    }
    public void setup(String height) {
        if(height != TargetHeights.INTAKE && height != TargetHeights.LOW && soOne.get() && !soTwo.get()) {
            run(TargetHeights.INTAKE);
            if(timestamp == -1.0) {
                timestamp = Timer.getFPGATimestamp();
            }
        }
        if(height != TargetHeights.LOW) {
            if(!soOne.get()) {
                soOne.set(true);
            }
            if(soTwo.get()) {
                soTwo.set(false);
            }
        }
    }
    public boolean donePrepping(String height) {
        return (height == TargetHeights.LOW) || (timestamp >= 0.0 && Timer.getFPGATimestamp() - timestamp >= 0.625 && soOne.get() && !soTwo.get());
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
        //return 0.2678; // 0.325
        return dist.getDouble(defaultDist);
    }
    public void simulationPeriodic() {

    }
    public void inOut(boolean in) {
        motors[0].run(ConerConstants.inOutVelocity * (in ? -1 : 1));
        motors[1].run(ConerConstants.inOutVelocity * (in ? 1 : -1));
    }

    // CONSTANTS
    private static final Map<String, TesterSetting[]> percents = Map.of(
    TargetHeights.LOW, new TesterSetting[] {
        new TesterSetting(0.11), new TesterSetting(0.11)
    }, TargetHeights.MID, new TesterSetting[] {
        new TesterSetting(0.12), new TesterSetting(0.45)//0.445
    }, TargetHeights.HIGH, new TesterSetting[] {
        new TesterSetting(0.22), new TesterSetting(1.0)
    }, TargetHeights.INTAKE, new TesterSetting[] {
        new TesterSetting(-0.2), new TesterSetting(-0.2)
    });

    private static final Map<String, TesterSetting[]> neoVelocities = Map.of(
    TargetHeights.LOW, new TesterSetting[] {
        new TesterSetting(true, 2500.0), new TesterSetting(true, 2500.0)
    }, TargetHeights.MID, new TesterSetting[] {
        new TesterSetting(true, 2500.0), new TesterSetting(true, 2500.0)//0.14, -0.36
    }, TargetHeights.HIGH, new TesterSetting[] {
        new TesterSetting(true, 5000.0), new TesterSetting(true, 5000.0)
    }, TargetHeights.INTAKE, new TesterSetting[] {
        new TesterSetting(true, -2500.0), new TesterSetting(true, -2500.0)
    });

    private static final Map<String, TesterSetting[]> falconVelocities = Map.of(
    TargetHeights.LOW, new TesterSetting[] {
        new TesterSetting(true, 10000.0), new TesterSetting(true, 10000.0)
    }, TargetHeights.MID, new TesterSetting[] {
        new TesterSetting(true, 10000.0), new TesterSetting(true, 10000.0)//0.14, -0.36
    }, TargetHeights.HIGH, new TesterSetting[] {
        new TesterSetting(true, 20000.0), new TesterSetting(true, 20000.0)
    }, TargetHeights.INTAKE, new TesterSetting[] {
        new TesterSetting(true, -10000.0), new TesterSetting(true, -10000.0)
    });
    public void periodic() {
        //System.out.println("cone distance: " + getLimelightDistance(TargetHeights.MID));
    }
}