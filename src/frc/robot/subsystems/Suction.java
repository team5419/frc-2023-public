package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;

public class Suction extends SubsystemBase implements GenericShootIntake {
    private Solenoid suctioner;
    private EverybotArm arm;
    public Suction(EverybotArm arm) {
        this.arm = arm;
        suctioner = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
        suctioner.set(false);
    }
    private void shoot() {
        suctioner.set(true);
    }
    public void shootHigh() {shoot();};
    public void shootMid() {shoot();};
    public void shootLow() {shoot();};
    public void intake() {}
    public void stop() {
        suctioner.set(false);
        arm.gotoPosition(Arm.inPosition);
    }
    public void setup(int height) {
        arm.gotoPosition(Arm.outPosition);
    }
}
