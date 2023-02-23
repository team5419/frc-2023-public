package frc.robot.subsystems;

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
import frc.robot.subsystems.test.TesterSubsystem;
import frc.robot.Util;
import edu.wpi.first.wpilibj.AnalogInput;

public class Cuber extends TesterSubsystem implements GenericShootIntake {
    private Solenoid soOne;
    //private Solenoid soTwo;
    private AnalogInput sensor;
    private Double startingPoint;
    public Cuber(PneumaticHub hub, boolean velocityControl) {
        super("Cube Shooter", new TesterMotor[] {
            new TesterNeo("Main", Util.setUpMotor(
                new CANSparkMax(Ports.intake, MotorType.kBrushless), true, false
            )),
            new TesterNeo("Indexer", Util.setUpMotor(
                new CANSparkMax(Ports.indexer, MotorType.kBrushless), true, true
            ))
        }, velocityControl ? CubeShooterConstants.velocities : CubeShooterConstants.percents);
        startingPoint = null;
        soOne = hub.makeSolenoid(Ports.cuberSolenoidA);
        soOne.set(false);
        //soTwo = hub.makeSolenoid(Ports.cuberSolenoidB);
        //soTwo.set(false);

        sensor = new AnalogInput(Ports.cuberSensor);

        ShuffleboardTab main = Shuffleboard.getTab("Master");
        main.addNumber("Cuber sensor", () -> getSensorValue()).withSize(1, 1).withPosition(2, 1);
        //main.addNumber("Backwards setpoint", () -> startingPoint == null ? 0.0 : startingPoint);
        main.addNumber("Cuber velocity", () -> motors[0].getVelocity());
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
        if(height == TargetHeights.FAR) {
            this.setup(TargetHeights.INTAKE);
            this.shoot(TargetHeights.INTAKE); // special setting
        } else {
            super.stop();
        }
    }
    public SubsystemBase subsystem() {return this;}
    public void setup(String height) {
        System.out.println("prep");
        if(height == TargetHeights.INTAKE) {
            soOne.set(true);
            //soTwo.set(false);
        } else {
            double pos = motors[1].getPosition();
            motors[1].run(CubeShooterConstants.indexerSlowBackwardsSpeed);
            // if(startingPoint == null) {
            //     startingPoint = pos;
            // } else {
                runSingle(height, 0);
            //     if(pos - startingPoint > 2) {
            //         motors[1].run(0.0);
            //     }
            // }
        }
    }
    public boolean donePrepping(String height) {
        return true;//(motors[0].getVelocity() >= CubeShooterConstants.measuredVelocities.get(height));
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
            return 2.4;
        }
        return 1.926;
        //return 1.84;
    }
    public final double getLimelightDistance(String height) {
        return -1.0;
    }
    public double getOffset() {
        double val = getSensorValue();
        if(val < CubeShooterConstants.sensorThresholdLeft) {
            return CubeShooterConstants.adjustmentLeft;
        }
        if(val > CubeShooterConstants.sensorThresholdRight) {
            return CubeShooterConstants.adjustmentRight;
        }
        return 0.0;
    }
}