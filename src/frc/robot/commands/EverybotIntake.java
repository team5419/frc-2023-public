package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EverybotArm;

public class EverybotIntake extends CommandBase {
    boolean reverse;
    EverybotArm arm;

    public EverybotIntake(EverybotArm _arm, boolean _reverse) {
        reverse = _reverse;
        arm = _arm;
        // addRequirements(arm);
    }

    public void initialize() {
        arm.start(reverse);
    }  

    public void execute() {
        
    }

    public boolean isFinished() {
        return true;
    }

    public void end(boolean interrupted) {
        arm.stop();
    }
}
