package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.Constants.IntakeConstants;

public class Shoot extends CommandBase {
    private Intake intake;
    private Indexer indexer;
    private Timer timer;

    public Shoot(Intake intake, Indexer indexer) {
        this.intake = intake;
        this.indexer = indexer;
        this.timer = new Timer();
    }
    public void initialize() {
        timer.reset();
        timer.start();
    }
    public void execute() {
        double speed = intake.getVelocity();
        if(/*Math.abs(speed) >= Math.abs(IntakeConstants.outtakeSpeed)*/timer.get() > 1.0) {
            indexer.run();
        }
        ///double output = IntakeConstants.outtakePID.calculate(speed, IntakeConstants.outtakeSpeed);
        intake.run();
    }
    public boolean isFinished() {
        return false;
    }
    public void end(boolean interrupted) {
        intake.stop();
        indexer.stop();
    }
}
