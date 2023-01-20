package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Suction;

public class SuctionIn extends CommandBase {
    private Suction suction;

    public SuctionIn(Suction _suction){
        this.suction = _suction;
        // addRequirements(this.suction);
    }

    public void initialize() {
        this.suction.airIn();
    }

    public void execute() {
    }

    public boolean isFinished() {
        return true;
    }
    public void end(boolean interrupted) {
    }
}
