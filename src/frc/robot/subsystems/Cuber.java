package frc.robot.subsystems;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CubeShooterConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.TargetHeights;
import frc.robot.classes.PID;
import frc.robot.subsystems.test.TesterFalcon;
import frc.robot.subsystems.test.TesterMotor;
import frc.robot.subsystems.test.TesterNeo;
import frc.robot.subsystems.test.TesterSetting;
import frc.robot.subsystems.test.TesterSubsystem;
import frc.robot.Util;
import edu.wpi.first.wpilibj.AnalogInput;

public class Cuber extends TesterSubsystem implements GenericShootIntake {
    public final double down = -565.0; // 1200
    public final double shotSetpoint = -2724.0; // 3408
    public final double up = -2724.0;
    public final double lowShot = -1565; // 2200
    
    private final PID lifterPID = new PID(0.5, 0.0, 0.0);
    private AnalogInput sensor;
    private boolean velocity;
    public double offset;
    private TalonFX lifter;
    private CANCoder cancoder;
    public double state;
    public Cuber(boolean velocityControl) {
        super("Cube Shooter", new TesterMotor[] {
            new TesterNeo("Indexer", Util.setUpMotor(
                new CANSparkMax(Ports.indexer, MotorType.kBrushless), false, true
            )).configurePID(CubeShooterConstants.upPID),
            new TesterFalcon("Main", Util.setUpMotor(
                new TalonFX(Ports.intake), false, true
            )).configurePID(CubeShooterConstants.upPID)
        }, velocityControl ? velocities : percents);
        
        lifter = new TalonFX(Ports.lifter, "canivore");
        TalonFXConfiguration tconfig = new TalonFXConfiguration();
        tconfig.remoteFilter0.remoteSensorDeviceID = Ports.lifterCancoder;
        tconfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        
        lifter.configAllSettings(tconfig);
        //Util.setUpMotor(lifter, false, false);
        lifter.setSensorPhase(true);
        lifter.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);
        lifter.configMotionCruiseVelocity(3200.0);
        lifter.configMotionAcceleration(9600.0 * 4);
        lifter.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40.0, 0.0, 0.0));
        lifter.config_kP(0, lifterPID.p);
        lifter.config_kI(0, lifterPID.i);
        lifter.config_kD(0, lifterPID.d);
        state = up;
        cancoder = new CANCoder(Ports.lifterCancoder, "canivore"); 
        CANCoderConfiguration config = new CANCoderConfiguration();
        
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        cancoder.configAllSettings(config, 100);
        cancoder.setPositionToAbsolute(100);
        
        
    
        offset = 1;
        velocity = velocityControl;

        sensor = new AnalogInput(Ports.cuberSensor);

        ShuffleboardTab main = Shuffleboard.getTab("Master");
        main.addNumber("Deploy position", lifter::getSelectedSensorPosition).withSize(1, 1);
        main.addNumber("Cuber sensor", this::getSensorValue).withSize(1, 1).withPosition(2, 1);
        //main.addNumber("Backwards setpoint", () -> startingPoint == null ? 0.0 : startingPoint);
        main.addNumber("Cuber velocity", motors[1]::getVelocity);
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
    public void setSpeed(String height) {
        motors[0].run(CubeShooterConstants.indexerSlowBackwardsSpeed);
        runSingle(height, 1);
    }
    public void runIntake() {
        run(TargetHeights.INTAKE);
    }
    public void shoot(String height) {
        run(height);
        if(height == TargetHeights.INTAKE) {
            state = down;
        }
    }
    public void stop(String height) {
            super.stop();
                state = up;  
    }
    public SubsystemBase subsystem() {return this;}
    public void setup(String height) {
        if(height != TargetHeights.INTAKE) {
            motors[0].run(CubeShooterConstants.indexerSlowBackwardsSpeed);
            runSingle(height, 1);
                state = (height == TargetHeights.LOW || height == TargetHeights.FAR) ? lowShot : shotSetpoint;
            
        }
    }
    public void setup(double setpoint) {
        motors[0].run(CubeShooterConstants.indexerSlowBackwardsSpeed);
        runVelocity(1, setpoint);
    }
    public boolean donePrepping(String height) {
        if(height == TargetHeights.LOW) {
            return Math.abs(lifter.getSelectedSensorPosition() - lowShot) < 150.0;
        }
        return (height == TargetHeights.FAR && motors[1].getVelocity() >= 13800.0) || (velocity ? (Math.abs(motors[1].getVelocity() - velocities.get(height)[1].getSetpoint()) <= 75.0)
        : (motors[1].getVelocity() >= CubeShooterConstants.measuredVelocities.get(height)));
    }
    public boolean donePrepping(double velocity) {
        return Math.abs(motors[1].getVelocity() - velocity) <= 75.0;
    }
    public void periodic() {
        double diff = lifter.getSelectedSensorPosition();
        if(diff <= -2400.0 && state == up) {
            //lifter.set(ControlMode.PercentOutput, 0.0);
            
        } else {
           // lifter.set(ControlMode.MotionMagic, state);
        }
        System.out.println(diff);
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
            return 2.25;
        }
        return 2.064;
        //return 1.84;
    }
    public double getOffset() {
        //double val = getSensorValue();
        if(/*(val < 400 && offset == 0) || val < CubeShooterConstants.sensorThresholdLeft*/offset == 0) {
            return CubeShooterConstants.adjustmentLeft;
        }
        if(/*(val < 400 && offset == 2) || val > CubeShooterConstants.sensorThresholdRight*/offset == 2) {
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
        new TesterSetting(1.0), new TesterSetting(0.13)
    }, TargetHeights.MID, new TesterSetting[] {
        new TesterSetting(1.0), new TesterSetting(true, 3300.0)//0.14, -0.36
    }, TargetHeights.HIGH, new TesterSetting[] {
        new TesterSetting(1.0), new TesterSetting(true, 7500.0)
    }, TargetHeights.INTAKE, new TesterSetting[] {
        new TesterSetting(-0.35), new TesterSetting(-0.60)
    }, TargetHeights.FAR, new TesterSetting[] {
        new TesterSetting(1.0), new TesterSetting(0.77)
    });
}