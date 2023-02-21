package frc.robot.subsystems;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
    public boolean isOut;
    public boolean enabled;
    public EverybotArm(boolean usePID) {
        this.usePID = usePID;
        this.isOut = false;
        this.enabled = false;
        leftArm = new TalonFX(Ports.everyArm0);
        Util.setUpMotor(leftArm, false, true, EverybotArmConstants.PID, true, 1.0);
        leftArm.configMotionAcceleration(40000, 100);
        leftArm.configMotionCruiseVelocity(25000, 100);
    
        rightArm = new TalonFX(Ports.everyArm1);
        Util.setUpMotor(rightArm, false, false, EverybotArmConstants.PID, true, 1.0);
        rightArm.configMotionAcceleration(40000, 100);
        rightArm.configMotionCruiseVelocity(25000, 100);
        rightArm.follow(leftArm); // hopefully this works

        ShuffleboardTab tab = Shuffleboard.getTab("Master");
        tab.addNumber("Left arm position", () -> leftArm.getSelectedSensorPosition());
        tab.addNumber("Right arm position", () -> rightArm.getSelectedSensorPosition());
        tab.addNumber("arm motion control vel", () -> leftArm.getActiveTrajectoryVelocity());
    }

    public void resetEncoders() {
        leftArm.setSelectedSensorPosition(0.0);
        rightArm.setSelectedSensorPosition(0.0);
    }

    public void move(double speed) {
        System.out.println("setting output to " + speed);
        leftArm.set(ControlMode.PercentOutput, speed);
    }

    public void periodic() {
        if(usePID && enabled) {
            if(isOut) {
                if(leftArm.getSelectedSensorPosition() > EverybotArmConstants.realOutPosition) {
                    leftArm.set(ControlMode.Velocity, 0.0);
                } else {
                    leftArm.set(ControlMode.MotionMagic, EverybotArmConstants.outPosition);
                }
            } else {
                if(leftArm.getSelectedSensorPosition() < EverybotArmConstants.realInPosition) {
                    leftArm.set(ControlMode.Velocity, 0.0);
                } else {
                    leftArm.set(ControlMode.MotionMagic, EverybotArmConstants.inPosition);
                }
            }
        }
    }

    public void simulationPeriodic() {

    }
}