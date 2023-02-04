package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EverybotArm;
import frc.robot.subsystems.Swerve;
import frc.robot.Util;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Drive;

public class ManualMoveArm extends CommandBase {
    XboxController codriver;
    EverybotArm arm;

    public ManualMoveArm(EverybotArm arm, XboxController codriver){
        this.arm = arm;
        this.codriver = codriver;
        addRequirements(arm);
    }

    public void initialize() {

    }
    public void execute() {
        arm.move(Util.deadband(codriver.getLeftY(), 0.1) * Arm.moveSpeed);
    }
    public boolean isFinished() {
        return false;
    }
    public void end(boolean interrupted) {
        arm.move(0.0);
    }
}
