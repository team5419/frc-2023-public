package frc.robot.modules;
import frc.robot.Constants.Drive;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix.motorcontrol.DemandType;
import frc.robot.Util;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;

public class SwerveModule implements ISwerveModule {
    private TalonFX driveMotor;
    private CANSparkMax turnMotor;
    private CANCoder turnEncoder;
    private double offset;
    private double lastTurnOutput;
    private double lastPercentOutput;
    public SwerveModule(ModuleInfo info, int num) {
        this.offset = info.offset;
        this.lastTurnOutput = 0.0;
        this.lastPercentOutput = 0.0;
        this.driveMotor = new TalonFX(info.driverPort, "canivore");
        this.turnMotor = new CANSparkMax(info.turnerPort, MotorType.kBrushless);
          turnMotor.restoreFactoryDefaults();
          turnMotor.setIdleMode(IdleMode.kCoast);
          turnMotor.setInverted(info.turnInverted);
          turnMotor.setSmartCurrentLimit(30);
          turnMotor.setClosedLoopRampRate(1.0);
          turnMotor.setControlFramePeriodMs(50);
          turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);
        this.turnEncoder = new CANCoder(info.cancoderPort, "canivore");
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.sensorCoefficient = Math.PI / 2048.0;
        config.unitString = "rad";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        this.turnEncoder.configAllSettings(config, 100);
        this.turnEncoder.setPositionToAbsolute(100);
            driveMotor.configFactoryDefault(100);
            driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40.0, 0.0, 0.0), 100);

            driveMotor.setSensorPhase(false);
            driveMotor.setInverted(info.driveInverted);

            driveMotor.config_kP( 0, Drive.DrivePID.p , 100 );
            driveMotor.config_kI( 0, Drive.DrivePID.i , 100 );
            driveMotor.config_kD( 0, Drive.DrivePID.d , 100 );
            driveMotor.config_kF( 0, 0.0 , 100 );

            driveMotor.setSelectedSensorPosition(0.0, 0, 100);

            driveMotor.configVoltageCompSaturation(12.0, 100);
            driveMotor.enableVoltageCompensation(true);
            driveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, 100);
            driveMotor.setControlFramePeriod(ControlFrame.Control_3_General, 10);

            driveMotor.setNeutralMode(NeutralMode.Coast);

            driveMotor.configClosedLoopPeakOutput(0, 0.1, 100);
            ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
            ShuffleboardLayout layout = tab.getLayout("Module ${num}", BuiltInLayouts.kList).withPosition(num + 1, 0).withSize(1, 4);
        layout.addNumber("angle", () -> { return getTurn().getRadians(); });
        layout.addNumber("drive", () -> { return getDrive(); });
        layout.addNumber("desired angle output", () -> { return this.lastTurnOutput; });
        layout.addNumber("desired percent output", () -> { return this.lastPercentOutput; });
    }

  public Rotation2d getTurn() {
        return new Rotation2d(turnEncoder.getPosition() - this.offset);
  }

  public double getDrive() {
    return driveMotor.getSelectedSensorVelocity(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(Util.nativeUnitsToMetersPerSecond(getDrive()), getTurn());
  }

  private SwerveModuleState optimize(SwerveModuleState state, Rotation2d turn, boolean slow) {
    double stateRadians = state.angle.getRadians();
    long a = Math.round((turn.getRadians() - stateRadians) / Math.PI);
    return new SwerveModuleState(
        state.speedMetersPerSecond * (a % 2 == 0L ? 1.0 : -1.0) * (slow ? Drive.slow : 1.0),
        new Rotation2d(a * Math.PI + stateRadians)
    );
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean preventTurn, boolean slow, boolean pid) {
    Rotation2d turn = getTurn();
    SwerveModuleState state = this.optimize(desiredState, turn, slow);
    double driveFeedForward = Drive.feedForward.calculate(state.speedMetersPerSecond);

    if(pid || slow) {
      this.lastPercentOutput = Util.metersPerSecondToNativeUnits(state.speedMetersPerSecond);
      driveMotor.set(ControlMode.Velocity, this.lastPercentOutput, DemandType.ArbitraryFeedForward, driveFeedForward);
    } else {
      this.lastPercentOutput = state.speedMetersPerSecond / Drive.maxVelocity;
      driveMotor.set(ControlMode.PercentOutput, this.lastPercentOutput);
    }
    
    if(preventTurn) {
      turnMotor.setVoltage(0.0);
      return;
    }
    double newTurnOutput = Drive.turnController.calculate(turn.getRadians(), state.angle.getRadians());
    this.lastTurnOutput = state.angle.getRadians();
    
    turnMotor.setVoltage(newTurnOutput);
  }

  public void simulationPeriodic(double dt) {}

  public void resetEncoders() {
    driveMotor.setSelectedSensorPosition(0.0);
    turnEncoder.setPosition(0.0);
  }

  public void setBrakeMode(boolean on) {
    driveMotor.setNeutralMode(on ? NeutralMode.Brake : NeutralMode.Coast);
    turnMotor.setIdleMode(on ? IdleMode.kBrake : IdleMode.kCoast); 
  }

  public void test() {
    driveMotor.set(ControlMode.PercentOutput, 0.1);
  }
}