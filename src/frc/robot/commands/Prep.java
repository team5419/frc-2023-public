package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Coner;
import frc.robot.subsystems.GenericShootIntake;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.TargetHeights;
public class Prep extends CommandBase {
    private Swerve drivetrain;
    private GenericShootIntake coneShooter;
    private GenericShootIntake cubeShooter;
    private boolean isCone;
    private int height;
    public Prep(GenericShootIntake coneShooter, GenericShootIntake cubeShooter, Swerve drivetrain) {
        this.drivetrain = drivetrain;
        this.coneShooter = coneShooter;
        this.cubeShooter = cubeShooter;
        addRequirements(coneShooter.subsystem());
        addRequirements(cubeShooter.subsystem());
        isCone = false;
        height = 0;;
    }
    public void initialize() {
        isCone = drivetrain.currentNum != 1;
        height = drivetrain.currentHeight; 
        if(isCone) {
            coneShooter.setup(TargetHeights.heights[height], true);
        } else {
            cubeShooter.setup(TargetHeights.heights[height], true);
        }
    }
    public void execute() {
        if(isCone) {
            coneShooter.setup(TargetHeights.heights[height], false);
        } else {
            cubeShooter.setup(TargetHeights.heights[height], false);
        }
    }
    public boolean isFinished() {
        return false;
    }
    public void end(boolean interrupted) {
    }
}