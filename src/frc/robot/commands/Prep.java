package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GenericShootIntake;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.Drive;
import frc.robot.Util;
public class Prep extends CommandBase {
    private Swerve drivetrain;
    private GenericShootIntake coneShooter;
    private GenericShootIntake cubeShooter;
    public Prep(GenericShootIntake coneShooter, GenericShootIntake cubeShooter, Swerve drivetrain) {
        this.drivetrain = drivetrain;
        this.coneShooter = coneShooter;
        this.cubeShooter = cubeShooter;
    }
    public void initialize() {
        boolean isCone = (drivetrain.currentNum - 1) % 3 != 0;
        int height = drivetrain.currentHeight; 
        if(isCone) {
            coneShooter.setup(height);
        } else {
            cubeShooter.setup(height);
        }
    }
    public void execute() {}
    public boolean isFinished() {
        return false;
    }
    public void end(boolean interrupted) {
    }
}