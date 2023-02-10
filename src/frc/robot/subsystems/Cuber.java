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
    private AnalogInput sensor;
    public Cuber(PneumaticHub hub, boolean velocityControl) {
        super("Cube Shooter", new TesterMotor[] {
            new TesterNeo("Main", Util.setUpMotor(
                new CANSparkMax(Ports.intake, MotorType.kBrushless), false, false
            )),
            new TesterNeo("Indexer", Util.setUpMotor(
                new CANSparkMax(Ports.indexer, MotorType.kBrushless), false, false
            ))
        }, velocityControl ? CubeShooterConstants.velocities : CubeShooterConstants.percents);

        soOne = hub.makeSolenoid(Ports.cuberSolenoid);
        soOne.set(false);

        sensor = new AnalogInput(Ports.cuberSensor);

        ShuffleboardTab main = Shuffleboard.getTab("Master");
        main.addNumber("Cuber sensor", () -> getSensorValue()).withSize(1, 1).withPosition(2, 1);
    }
    public double getSensorValue() {
        return sensor.getValue();
    }
    public void shoot(String height) {
        run(height);
    }
    public void stop(String height) {
        if(height == TargetHeights.INTAKE) {
            soOne.set(false);
        }
        super.stop();
    }
    public SubsystemBase subsystem() {return this;}
    public void setup(String height) {
        motors[1].run(CubeShooterConstants.indexerSlowBackwardsSpeed);
        runSingle(height, 0);
        if(height == TargetHeights.INTAKE) {
            soOne.set(true);
        }
    }
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
    public final double getAngle() {
        return 0.0;
    }
    public final double getDistance(String height) {
        return 2.17;
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