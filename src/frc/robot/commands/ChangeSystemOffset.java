package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TargetHeights;
import frc.robot.subsystems.Coner;
import frc.robot.subsystems.Cuber;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.test.TesterSubsystem;

public class ChangeSystemOffset extends CommandBase {
    private int num;
    private double change;
    private Cuber cuber;
    private Coner coner;
    private Swerve swerve;
    public ChangeSystemOffset(int num, double change, Cuber cuber, Coner coner, Swerve swerve) {
        this.num = num;
        this.change = change;
        this.cuber = cuber;
        this.coner= coner;
        this.swerve = swerve;
    }
    public void initialize() {
        TesterSubsystem system = cuber;
		if(swerve.usingCones) {
			system = coner;
		}
		system.changeMotorOffset(TargetHeights.heights[swerve.currentHeight], num, change);
    }
    public boolean isFinished() {
        return true;
    }
}
