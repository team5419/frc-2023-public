package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Suction;

public class SuctionIn extends CommandBase {
    private Suction suction;

    public SuctionIn(Suction _suction){
        this.suction = _suction;
        addRequirements(suction);
    }

    public void initialize() {

    }
    public void execute() {
        drivetrain.drive(driver.getLeftY(), driver.getRightX());
    }

    public boolean isFinished() {
        return true;
    }
    public void end(boolean interrupted) {
        drivetrain.drive(0.0, 0.0);
    }
}
