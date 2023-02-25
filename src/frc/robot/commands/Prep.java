package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GenericShootIntake;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.TargetHeights;
public class Prep extends CommandBase {
    private Swerve drivetrain;
    private GenericShootIntake coneShooter;
    private GenericShootIntake cubeShooter;
    private boolean isCone;
    private int height;
    private String overrideHeight;
    public Prep(GenericShootIntake coneShooter, GenericShootIntake cubeShooter, Swerve drivetrain, String overrideHeight) {
        this.drivetrain = drivetrain;
        this.coneShooter = coneShooter;
        this.cubeShooter = cubeShooter;
        addRequirements(coneShooter.subsystem());
        addRequirements(cubeShooter.subsystem());
        isCone = false;
        height = 0;
        this.overrideHeight = overrideHeight;
    }
    public void initialize() {
        isCone = drivetrain.usingCones;
        height = drivetrain.currentHeight; 
    }
    public void execute() {
        if(isCone) {
            coneShooter.setup(overrideHeight == null ? TargetHeights.heights[height] : overrideHeight);
        } else {
            cubeShooter.setup(overrideHeight == null ? TargetHeights.heights[height] : overrideHeight);
        }
    }
    public boolean isFinished() {
        return false;
    }
    public void end(boolean interrupted) {
    }
}