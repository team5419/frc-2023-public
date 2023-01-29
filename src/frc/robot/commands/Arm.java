package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EverybotArm;

public class Arm extends CommandBase {
    XboxController driver;
    EverybotArm arm;

    public Arm(EverybotArm _arm, XboxController _driver){
        driver = _driver;
        arm = _arm;
        // addRequirements(arm);
    }

    public void initialize() {

    }

    public void execute() {
        arm.arm(driver.getRightTriggerAxis() - driver.getLeftTriggerAxis());
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        
    }
}
