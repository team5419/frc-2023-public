package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CubeShooterConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.TargetHeights;
import frc.robot.classes.PID;
import frc.robot.subsystems.test.TesterMotor;
import frc.robot.subsystems.test.TesterNeo;
import frc.robot.subsystems.test.TesterSetting;
import frc.robot.subsystems.test.TesterSubsystem;
import frc.robot.Util;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;

public class Cuber extends TesterSubsystem implements GenericShootIntake {
    private final double down = -1250.0;
    private final double up = -3262.0;
    private enum State {
        DOWN, UP, HOLDUP
    };
    private final PID lifterPID = new PID(0.5, 0.0, 0.0);
    private AnalogInput sensor;
    private boolean velocity;
    public double offset;
    private TalonFX lifter;
    private CANCoder cancoder;
    private State state;
    public Cuber(boolean velocityControl) {
        super("Cube Shooter", new TesterMotor[] {
            new TesterNeo("Indexer", Util.setUpMotor(
                new CANSparkMax(Ports.indexer, MotorType.kBrushless), false, true
            )).configurePID(CubeShooterConstants.upPID),
            new TesterNeo("Main", Util.setUpMotor(
                new CANSparkMax(Ports.intake, MotorType.kBrushless), true, false
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
        state = State.UP;
        cancoder = new CANCoder(Ports.lifterCancoder, "canivore"); 
        CANCoderConfiguration config = new CANCoderConfiguration();
        
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        cancoder.configAllSettings(config, 100);
        cancoder.setPositionToAbsolute(100);
        
        
    
        offset = 1;
        velocity = velocityControl;

        sensor = new AnalogInput(Ports.cuberSensor);

        ShuffleboardTab main = Shuffleboard.getTab("Master");
        main.addString("Cube state", () -> {
            switch(state) {
                case UP:
                    return "UP";
                case DOWN: return "DOWN";
                case HOLDUP: return "HOLD UP";
            }
            return "";
        });
        main.addNumber("Deploy position", () -> lifter.getSelectedSensorPosition()).withSize(1, 1);
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
        if(height == TargetHeights.INTAKE) {
            state = State.DOWN;
        }
    }
    public void stop(String height) {
            super.stop();
                state = State.UP;
            
    }
    public SubsystemBase subsystem() {return this;}
    public void setup(String height) {
        if(height != TargetHeights.INTAKE) {
            motors[0].run(CubeShooterConstants.indexerSlowBackwardsSpeed);
                runSingle(height, 1);
                state = State.HOLDUP;
        }
    }
    public boolean donePrepping(String height) {
        return velocity ? (Math.abs(motors[1].getVelocity() - velocities.get(height)[1].getSetpoint()) <= 125.0)
        : (motors[1].getVelocity() >= CubeShooterConstants.measuredVelocities.get(height));
    }
    public void periodic() {
        double setpoint = (state == State.DOWN) ? down : up;
        double diff = lifter.getSelectedSensorPosition();
        if(diff - 100.0 <= setpoint && state == State.UP) {
            lifter.set(ControlMode.PercentOutput, 0.0);
            
        } else {
            // double val = -lifterPID.calculate(diff, setpoint);
            // if(Math.abs(val) > 0.1) {
            //     val = 0.1 * Math.signum(val);
            // }
            lifter.set(ControlMode.MotionMagic, setpoint);
            //lifter.set(ControlMode.PercentOutput, val);
        }
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
        new TesterSetting(1.0), new TesterSetting(true, 585.0)
    }, TargetHeights.MID, new TesterSetting[] {
        new TesterSetting(1.0), new TesterSetting(true, 1755.0)//0.14, -0.36
    }, TargetHeights.HIGH, new TesterSetting[] {
        new TesterSetting(1.0), new TesterSetting(true, 2225.0)
    }, TargetHeights.INTAKE, new TesterSetting[] {
        new TesterSetting(-1.0), new TesterSetting(-0.35)
    }, TargetHeights.FAR, new TesterSetting[] {
        new TesterSetting(1.0), new TesterSetting(true, 4000.0)
    });
}