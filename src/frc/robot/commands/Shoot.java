package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TargetHeights;
import frc.robot.subsystems.GenericShootIntake;
import frc.robot.subsystems.Swerve;

public class Shoot extends CommandBase {
    private GenericShootIntake coneShooter;
    private GenericShootIntake cubeShooter;
    private Swerve drivetrain;
    private boolean isCone;
    private int height;
    public Shoot(GenericShootIntake coneShooter, GenericShootIntake cubeShooter, Swerve drivetrain) {
        this.coneShooter = coneShooter;
        this.cubeShooter = cubeShooter;
        this.drivetrain = drivetrain;
        isCone = false;
        height = 0;
        addRequirements(coneShooter.subsystem());
        addRequirements(cubeShooter.subsystem());
    }
    public void initialize() {
        isCone = drivetrain.currentNum != 1;
        height = drivetrain.currentHeight; 
    }
    public void execute() {
        System.out.println("going to shoot, height: " + height);
        GenericShootIntake shooter = isCone ? coneShooter : cubeShooter;
        shooter.shoot(TargetHeights.heights[height]);
    }
    public boolean isFinished() {
        return false;
    }
    public void end(boolean interrupted) {
        if(isCone) {
            coneShooter.stop(TargetHeights.heights[height]);
        } else {
            cubeShooter.stop(TargetHeights.heights[height]);
        }
    }
}
