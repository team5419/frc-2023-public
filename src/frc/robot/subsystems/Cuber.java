package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CubeShooterConstants;
import frc.robot.Constants.Ports;
import frc.robot.subsystems.test.TesterMotor;
import frc.robot.subsystems.test.TesterNeo;
import frc.robot.subsystems.test.TesterSubsystem;
import frc.robot.Util;

public class Cuber extends TesterSubsystem implements GenericShootIntake {
    public Cuber(boolean velocityControl) {
        super("Cube Shooter", new TesterMotor[] {
            new TesterNeo("Main", Util.setUpMotor(
                new CANSparkMax(Ports.intake, MotorType.kBrushless), false, false
            )),
            new TesterNeo("Indexer", Util.setUpMotor(
                new CANSparkMax(Ports.indexer, MotorType.kBrushless), false, false
            ))
        }, velocityControl ? CubeShooterConstants.velocities : CubeShooterConstants.percents);
    }
    public void shoot(String height) {
        runSingle(height, 1);
    }
    public void stop() {
        super.stop();
    }
    public SubsystemBase subsystem() {return this;}
    public void setup(String height) {
        motors[1].run(CubeShooterConstants.indexerSlowBackwardsSpeed);
        runSingle(height, 0);
    }
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
}