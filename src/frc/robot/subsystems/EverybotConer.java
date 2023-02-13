package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EverybotArmConstants;
import frc.robot.Constants.EverybotConeConstants;
import frc.robot.Constants.Ports;
import frc.robot.subsystems.test.TesterFalcon;
import frc.robot.subsystems.test.TesterMotor;
import frc.robot.subsystems.test.TesterSubsystem;
import frc.robot.Util;

public class EverybotConer extends TesterSubsystem implements GenericShootIntake {
    private EverybotArm arm;
    public EverybotConer(EverybotArm arm, boolean velocityControl) {
        super("Everybot Cone Shooter", new TesterMotor[] {
            new TesterFalcon("Main", Util.setUpMotor(
                new TalonFX(Ports.everyIntakeMotor, "canivore"), false, false
            ))
        }, velocityControl ? EverybotConeConstants.velocities : EverybotConeConstants.percents);
        this.arm = arm;
    }
    public void shoot(String height) {
        run(height);
    }
    public void setup(String height, boolean first) {
        if(first) {
            arm.gotoPosition(EverybotArmConstants.outPosition);
        }
    }
    public SubsystemBase subsystem() {return this;}
    public void stop(String height) {
        super.stop();
        arm.gotoPosition(EverybotArmConstants.inPosition);
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
}