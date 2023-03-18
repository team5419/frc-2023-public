package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.GenericShootIntake;
import frc.robot.Constants.TargetHeights;


public class SlightOutake extends CommandBase{
    
    private GenericShootIntake coner;
    private Timer timer;

    public SlightOutake(GenericShootIntake _coner){
        timer = new Timer();
        this.coner = _coner;
        addRequirements(coner.subsystem());
    }
    public void initialize(){
        //coner.setup(TargetHeights.heights[1]);
        timer.reset();
        timer.start();
        System.out.println("Started Timer");
        while (timer.get() < .08){
            System.out.println("Slighting");
            coner.shoot(TargetHeights.heights[4]);
        }
        coner.stop(TargetHeights.heights[1]);
    }
    public void execute(){
    }
    public boolean isFinished(){
        return false;
    }
    public void end(){
    }

}
