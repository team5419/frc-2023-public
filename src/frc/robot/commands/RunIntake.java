package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
    private Intake intake;
    private double speed;
    private double indexSpeed;
    private double time;
    private Timer timer;
    private Indexer indexer;
    public RunIntake(Intake intake, Indexer indexer, double speed, double indexSpeed, double time) {
        this.intake = intake;
        this.indexer = indexer;
        this.speed = speed;
        this.indexSpeed = indexSpeed;
        this.time = time;
        timer = new Timer();
        addRequirements(intake);
    }
    public RunIntake(Intake intake, Indexer indexer) {
        this.intake = intake;
        this.indexer = indexer;
        speed = IntakeConstants.intakeSpeed;
        indexSpeed = IntakeConstants.indexerIntakeSpeed;
        time = 0.0;
        timer = new Timer();
    }
    public void initialize() {
        if(time != 0.0) {
            timer.reset();
            timer.start();
        }
<<<<<<< HEAD
        intake.run(speed);
        indexer.run(indexSpeed);
=======
        intake.run(speed); 
>>>>>>> e1671c2ed58933bd695d6d35ddbaf33607ca688c
    }
    public void execute() {
    }
    public boolean isFinished() {
        return time != 0.0 && timer.get() >= time; //stops if time is not 0 and has surpassed the 
    }
    public void end(boolean interrupted) {
        intake.run(0.0);
        indexer.run(0.0);
    }
}
