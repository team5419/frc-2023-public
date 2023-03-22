package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Coner;
import frc.robot.subsystems.GenericShootIntake;
import frc.robot.Constants.TargetHeights;


public class SlightOutake extends CommandBase {
    private Coner coner;
    private Timer timer;

    public SlightOutake(Coner _coner){
        timer = new Timer();
        this.coner = _coner;
        addRequirements(coner.subsystem());
    }
    public void initialize() {
        timer.reset();
        timer.start();
    }
    public void execute(){
        coner.runPercentOutput(0, 0.06);
        coner.runPercentOutput(1, 0.06);
    }
    public boolean isFinished() {
        System.out.println(timer.get());
        return timer.get() > 0.05;
    }
    public void end(boolean interrupted) {
        coner.stop(TargetHeights.MID);
        timer.stop();
    }
}
