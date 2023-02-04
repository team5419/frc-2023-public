package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants.EverybotArmConstants;
import frc.robot.Constants.Ports;
import frc.robot.Util;

public class EverybotArm extends SubsystemBase {
    private TalonFX leftArm;
    private TalonFX rightArm;
    private boolean usePID;
    public EverybotArm(boolean usePID) {
        this.usePID = usePID;
        leftArm = new TalonFX(Ports.everyArm0);
        Util.setUpMotor(leftArm, EverybotArmConstants.PID, true, 1.0);
        rightArm = new TalonFX(Ports.everyArm1);
        Util.setUpMotor(rightArm, EverybotArmConstants.PID, true, 1.0);
        rightArm.follow(leftArm); // hopefully this works
    }

    public void gotoPosition(double ticks) {
        if(usePID) {
            leftArm.set(ControlMode.Position, ticks);
        }
    }

    public void move(double speed) {
        leftArm.set(ControlMode.PercentOutput, speed);
    }

    public void periodic() {

    }

    public void simulationPeriodic() {

    }
}