package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CubeShooterConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.TargetHeights;
import frc.robot.subsystems.test.TesterMotor;
import frc.robot.subsystems.test.TesterNeo;
import frc.robot.subsystems.test.TesterSetting;
import frc.robot.subsystems.test.TesterSubsystem;
import frc.robot.Util;
import edu.wpi.first.wpilibj.AnalogInput;

public class Cuber extends TesterSubsystem implements GenericShootIntake {
    private Solenoid soOne;
    //private Solenoid soTwo;
    private AnalogInput sensor;
    private Double startingPoint;
    private boolean velocity;
    public double offset;
    public Cuber(PneumaticHub hub, boolean velocityControl) {
        super("Cube Shooter", new TesterMotor[] {
            new TesterNeo("Indexer", Util.setUpMotor(
                new CANSparkMax(Ports.indexer, MotorType.kBrushless), false, true
            )).configurePID(CubeShooterConstants.upPID),
            new TesterNeo("Main", Util.setUpMotor(
                new CANSparkMax(Ports.intake, MotorType.kBrushless), true, false
            )).configurePID(CubeShooterConstants.upPID)
        }, velocityControl ? velocities : percents);
        startingPoint = null;
        offset = 1;
        velocity = velocityControl;
        soOne = hub.makeSolenoid(Ports.cuberSolenoidA);
        soOne.set(false);
        //soTwo = hub.makeSolenoid(Ports.cuberSolenoidB);
        //soTwo.set(false);

        sensor = new AnalogInput(Ports.cuberSensor);

        ShuffleboardTab main = Shuffleboard.getTab("Master");
        main.addNumber("Cuber sensor", () -> getSensorValue()).withSize(1, 1).withPosition(2, 1);
        //main.addNumber("Backwards setpoint", () -> startingPoint == null ? 0.0 : startingPoint);
        main.addNumber("Cuber velocity", () -> motors[1].getVelocity());
        ShuffleboardTab tab = Shuffleboard.getTab("Shot Selection");
        tab.addBoolean("CUBE SENSOR MANUAL", () -> getSensorValue() < 400.0).withPosition(7, 3).withSize(1, 1);
        for(int i = 0; i < 3; i++) {
            int savedI = i;
            tab.addBoolean("col " + i, () -> (offset == savedI))
                .withSize(1, 1)
                .withPosition(3 + i, 3);
        }
    }
    public double getSensorValue() {
        return sensor.getValue();
    }
    public void shoot(String height) {
        run(height);
        startingPoint = null;
    }
    public void stop(String height) {
        if(height == TargetHeights.INTAKE) {
            soOne.set(false);
           //soTwo.set(true);
        }
            super.stop();
        
    }
    public SubsystemBase subsystem() {return this;}
    public void setup(String height) {
        System.out.println("prep");
        if(height == TargetHeights.INTAKE) {
            soOne.set(true);
            //soTwo.set(false);
        } else {
            //double pos = motors[0].getPosition();
            motors[0].run(CubeShooterConstants.indexerSlowBackwardsSpeed);
            // if(startingPoint == null) {
            //     startingPoint = pos;
            // } else {
                runSingle(height, 1);
            //     if(pos - startingPoint > 2) {
            //         motors[0].run(0.0);
            //     }
            // }
        }
    }
    public boolean donePrepping(String height) {
        return velocity ? (Math.abs(motors[1].getVelocity() - velocities.get(height)[1].getSetpoint()) <= 50.0)
        : (motors[1].getVelocity() >= CubeShooterConstants.measuredVelocities.get(height));
    }
    public void periodic() {
        //System.out.println(motors[1].getPosition());
    }
    public void simulationPeriodic() {

    }
    public final boolean prepsByDefault() {
        return true;
    }
    public final double getAngle() {
        return 0.0;
    }
    public final double getDistance(String height) {
        if(height == TargetHeights.HIGH) {
            return 2.139;
        }
        return 1.926;
        //return 1.84;
    }
    public final double getLimelightDistance(String height) {
        return -1.0;
    }
    public double getOffset() {
        double val = getSensorValue();
        if((val < 400 && offset == 0) || val < CubeShooterConstants.sensorThresholdLeft) {
            return CubeShooterConstants.adjustmentLeft;
        }
        if((val < 400 && offset == 2) || val > CubeShooterConstants.sensorThresholdRight) {
            return CubeShooterConstants.adjustmentRight;
        }
        return 0.0;
    }

    private static final Map<String, TesterSetting[]> percents = Map.of(
    TargetHeights.LOW, new TesterSetting[] {
        new TesterSetting(1.0), new TesterSetting(0.13)
    }, TargetHeights.MID, new TesterSetting[] {
        new TesterSetting(1.0), new TesterSetting(0.39)//0.14, -0.36
    }, TargetHeights.HIGH, new TesterSetting[] {
        new TesterSetting(1.0), new TesterSetting(0.46)
    }, TargetHeights.INTAKE, new TesterSetting[] {
        new TesterSetting(-0.2), new TesterSetting(-0.55)
    }, TargetHeights.FAR, new TesterSetting[] {
        new TesterSetting(1.0), new TesterSetting(0.97)
    });

    private static final Map<String, TesterSetting[]> velocities = Map.of(
    TargetHeights.LOW, new TesterSetting[] {
        new TesterSetting(1.0), new TesterSetting(true, 585.0)
    }, TargetHeights.MID, new TesterSetting[] {
        new TesterSetting(1.0), new TesterSetting(true, 1755.0)//0.14, -0.36
    }, TargetHeights.HIGH, new TesterSetting[] {
        new TesterSetting(1.0), new TesterSetting(true, 2225.0)
    }, TargetHeights.INTAKE, new TesterSetting[] {
        new TesterSetting(-0.2), new TesterSetting(-0.55)
    }, TargetHeights.FAR, new TesterSetting[] {
        new TesterSetting(1.0), new TesterSetting(true, 4300.0)
    });
}