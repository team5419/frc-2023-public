package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EverybotArmConstants;
import frc.robot.Constants.TargetHeights;

public class Suction extends SubsystemBase implements GenericShootIntake {
    private Solenoid suctioner;
    private EverybotArm arm;
    public Suction(EverybotArm arm) {
        this.arm = arm;
        suctioner = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
        suctioner.set(false);
    }
    public void shoot(String height) {
        if(height != TargetHeights.INTAKE) {
            suctioner.set(true);
        }
    }
    public SubsystemBase subsystem() {return this;}
    public void stop(String height) {
        suctioner.set(false);
        arm.gotoPosition(EverybotArmConstants.inPosition);
    }
    public void setup(String height) {
        arm.gotoPosition(EverybotArmConstants.outPosition);
    }
}
