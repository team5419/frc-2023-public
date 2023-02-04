package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GenericShootIntake;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.TargetHeights;
public class Prep extends CommandBase {
    private Swerve drivetrain;
    private GenericShootIntake coneShooter;
    private GenericShootIntake cubeShooter;
    public Prep(GenericShootIntake coneShooter, GenericShootIntake cubeShooter, Swerve drivetrain) {
        this.drivetrain = drivetrain;
        this.coneShooter = coneShooter;
        this.cubeShooter = cubeShooter;
        addRequirements(coneShooter.subsystem());
        addRequirements(cubeShooter.subsystem());
    }
    public void initialize() {
        boolean isCone = (drivetrain.currentNum - 1) % 3 != 0;
        int height = drivetrain.currentHeight; 
        if(isCone) {
            coneShooter.setup(TargetHeights.heights[height]);
        } else {
            cubeShooter.setup(TargetHeights.heights[height]);
        }
    }
    public void execute() {}
    public boolean isFinished() {
        return true;
    }
    public void end(boolean interrupted) {
    }
}