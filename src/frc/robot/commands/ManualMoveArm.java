package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EverybotArm;
import frc.robot.Util;
import frc.robot.Constants.EverybotArmConstants;
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
        // if(codriver.getAButtonPressed()) {
        //     arm.resetEncoders();
        // } else if(codriver.getBButton()) {
        //     arm.goOut();
        // } else if(codriver.getXButton()) {
        //     arm.goIn();
        // } else {
            arm.move(Util.deadband(codriver.getLeftY(), 0.1) * EverybotArmConstants.moveSpeed);
        //}
    }
    public boolean isFinished() {
        return false;
    }
    public void end(boolean interrupted) {
        arm.move(0.0);
    }
}
