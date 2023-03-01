package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.TargetHeights;
import frc.robot.subsystems.test.TesterFalcon;
import frc.robot.subsystems.test.TesterMotor;
import frc.robot.subsystems.test.TesterSetting;
import frc.robot.subsystems.test.TesterSubsystem;
import frc.robot.Util;

public class EverybotConer extends TesterSubsystem implements GenericShootIntake {
    private EverybotArm arm;
    public EverybotConer(EverybotArm arm, boolean velocityControl) {
        super("Everybot Cone Shooter", new TesterMotor[] {
            new TesterFalcon("Main", Util.setUpMotor(
                new TalonFX(Ports.everyIntakeMotor, "canivore"), false, false
            ))
        }, velocityControl ? velocities : percents);
        this.arm = arm;
    }
    public void shoot(String height) {
        run(height);
    }
    public void setup(String height) {
        arm.isOut = true;
    }
    public boolean donePrepping(String height) {
        return true;
    }
    public SubsystemBase subsystem() {return this;}
    public void stop(String height) {
        super.stop();
        arm.isOut = false;
    }
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
    public final double getAngle() {
        return 90.0;
    }
    public final double getOffset() {
        return 0.0;
    }
    public final double getDistance(String height) {
        return 1.84;
    }
    public final double getLimelightDistance(String height) {
        return 1.84;
    }
    public final boolean prepsByDefault() {
        return true;
    }
    private static final Map<String, TesterSetting[]> percents = Map.of(
    TargetHeights.LOW, new TesterSetting[] {
        new TesterSetting(-1.0)
    }, TargetHeights.MID, new TesterSetting[] {
        new TesterSetting(-1.0)
    }, TargetHeights.HIGH, new TesterSetting[] {
        new TesterSetting(-1.0)
    }, TargetHeights.INTAKE, new TesterSetting[] {
        new TesterSetting(0.25)
    });

    private static final Map<String, TesterSetting[]> velocities = Map.of(
    TargetHeights.LOW, new TesterSetting[] {
        new TesterSetting(true, -5000.0)
    }, TargetHeights.MID, new TesterSetting[] {
        new TesterSetting(true, -5000.0)
    }, TargetHeights.HIGH, new TesterSetting[] {
        new TesterSetting(true, -5000.0)
    }, TargetHeights.INTAKE, new TesterSetting[] {
        new TesterSetting(true, 1250.0)
    });
}