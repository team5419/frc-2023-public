package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
    }
    public void initialize() {
        isCone = (drivetrain.currentNum - 1) % 3 != 0;
        height = drivetrain.currentHeight; 
    }
    public void execute() {
        GenericShootIntake shooter = isCone ? coneShooter : cubeShooter;
        switch(height) {
            case 0:
                shooter.shootLow();
                break;
            case 1:
                shooter.shootMid();
                break;
            case 2:
                shooter.shootHigh();
                break;
        }
    }
    public boolean isFinished() {
        return false;
    }
    public void end(boolean interrupted) {
        if(isCone) {
            coneShooter.stop();
        } else {
            cubeShooter.stop();
        }
    }
}
