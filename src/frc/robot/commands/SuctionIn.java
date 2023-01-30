package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Suction;

import edu.wpi.first.wpilibj.Timer;

public class SuctionIn extends CommandBase {
    private Suction suction;
    private Timer timer;

    public SuctionIn(Suction _suction){
        this.suction = _suction;
        this.timer = new Timer();
        // addRequirements(this.suction);
    }

    public void initialize() {
        this.timer.reset();
        this.timer.start();
        this.suction.airIn();
    }

    public void execute() {
    }

    public boolean isFinished() {
        return this.timer.get() > 3.0;
    }
    public void end(boolean interrupted) {
        this.timer.stop();
    }
}
