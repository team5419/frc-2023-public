package frc.robot.subsystems;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.subsystems.test.TesterSetting;
import frc.robot.subsystems.test.TesterSubsystem;
import frc.robot.Util;

public class Coner extends TesterSubsystem implements GenericShootIntake {
    public Elevator elevator;
    private double timestamp;
    public Coner(Elevator elevator, boolean falcons, boolean velocityControl) {
        super("Cone Shooter", new TesterMotor[] {
            generateTesterMotor("Low motor", falcons, Ports.coneBottom, false),
            generateTesterMotor("High motor", falcons, Ports.coneTop, false)
        }, velocityControl ? (falcons ? falconVelocities : neoVelocities) : percents);
        this.elevator = elevator;
        timestamp = -1.0;
        ShuffleboardTab tab = Shuffleboard.getTab("Master");
        // dist = tab.add("Limelight dist", defaultDist)
        //     .withPosition(0, 3)
        //     .withSize(1, 1)
        //     .getEntry();
    }
    public static TesterMotor generateTesterMotor(String name, boolean falcons, int id, boolean inverted) {
        if(falcons) {
            TalonFX motor = new TalonFX(id, "canivore");
            Util.setUpMotor(motor, false, inverted, new PID(0.1, 0, 0), true, 1.0);
            
            SupplyCurrentLimitConfiguration current = new SupplyCurrentLimitConfiguration();
            current.currentLimit = 20.0;
            motor.configSupplyCurrentLimit(current);
            return new TesterFalcon(name, motor);
        }
        return new TesterNeo(name, Util.setUpMotor(new CANSparkMax(id, MotorType.kBrushless), inverted, true));
    }
    public void elevatorOut(String height) {
        elevator.state = Elevator.heights.get(height);
    }
    public void stopMotors() {
        super.stop();
    }
    public void resetTimestamp() {
        timestamp = Timer.getFPGATimestamp();
    }
    public void shoot(String height) {

            run(height);

        timestamp = -1.0;
    }
    public void stop(String height) {
        timestamp = -1.0;
        super.stop();
        elevator.state = Elevator.down;
    }
    public void setup(String height) {
        if(height != TargetHeights.INTAKE && height != TargetHeights.LOW && height != TargetHeights.FAR) {
            run(TargetHeights.INTAKE);
            if(timestamp == -1.0) {
                timestamp = Timer.getFPGATimestamp();
            }
        }
        elevator.state = Elevator.heights.get(height);
    }
    public boolean donePrepping(String height) {
        return (height == TargetHeights.LOW) || (height == TargetHeights.FAR && elevator.position() > 13000.0) || (timestamp >= 0.0 && Timer.getFPGATimestamp() - timestamp >= 0.4 && elevator.closeEnough());
    }
    public SubsystemBase subsystem() {return this;}
    public final double getAngle() {
        return 180.0;
    }
    public final boolean prepsByDefault() {
        return false;
    }
    public final double getLimelightDistance(String height) {
        return height == TargetHeights.MID ? 0.92 : 0.92;
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
        new TesterSetting(0.10), new TesterSetting(0.16)
    }, TargetHeights.MID, new TesterSetting[] {
        new TesterSetting(0.075), new TesterSetting(0.28)//0.445
    }, TargetHeights.HIGH, new TesterSetting[] {
        new TesterSetting(0.19), new TesterSetting(0.71)
    }, TargetHeights.INTAKE, new TesterSetting[] {
        new TesterSetting(-0.2), new TesterSetting(-0.2)
    }, TargetHeights.FAR, new TesterSetting[] {
        new TesterSetting(1.0), new TesterSetting(1.0)
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