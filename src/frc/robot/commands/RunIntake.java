package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
public class RunIntake extends CommandBase {
    private Intake intake;
    public RunIntake(Intake intake) {
        this.intake = intake;
    }
    public void initialize() {
        intake.run(1.0);
    }
    public void execute() {

    }
    public boolean isFinished() {
        return true;
    }
    public void end(boolean interrupted) {
        intake.run(0.0);
    }
}
