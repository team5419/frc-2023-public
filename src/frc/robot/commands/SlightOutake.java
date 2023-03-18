package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.GenericShootIntake;
import frc.robot.Constants.TargetHeights;


public class SlightOutake extends CommandBase {
    private GenericShootIntake coner;
    private Timer timer;

    public SlightOutake(GenericShootIntake _coner){
        timer = new Timer();
        this.coner = _coner;
        addRequirements(coner.subsystem());
    }
    public void initialize() {
        timer.reset();
        timer.start();
    }
    public void execute(){
        coner.shoot(TargetHeights.MID);
    }
    public boolean isFinished() {
        return timer.get() > 0.08;
    }
    public void end() {
        coner.stop(TargetHeights.MID);
        timer.stop();
    }
}
