package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EverybotArm;
import edu.wpi.first.wpilibj.Timer;

public class EverybotIntake extends CommandBase {
    private boolean reverse;
    private EverybotArm arm;
    private Timer timer;

    public EverybotIntake(EverybotArm _arm, boolean _reverse) {
        this.reverse = _reverse;
        this.arm = _arm;
        this.timer = new Timer();
    }

    public void initialize() {
        this.timer.reset();
        this.timer.start();
        this.arm.start(reverse);
    }  

    public void execute() {
        
    }

    public boolean isFinished() {
        return this.timer.get() > 2;
    }

    public void end(boolean interrupted) {
        arm.stop();
        timer.stop();
    }
}
