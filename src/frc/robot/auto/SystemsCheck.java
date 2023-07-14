package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TimedCommand;
import frc.robot.commands.shooting.RunIntake;
import frc.robot.commands.shooting.Shoot;
import frc.robot.subsystems.Coner;
import frc.robot.subsystems.Cuber;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve;

public class SystemsCheck extends SequentialCommandGroup {
    public SystemsCheck(Swerve swerve, Cuber cuber, Coner coner, Lights lights) {
        addCommands(
            new TimedCommand(2.5, () -> {
                swerve.drive(1.0, 0.0, 0.0, false, false, false);
            }, () -> { swerve.stop(); }),
            new TimedCommand(2.5, () -> {
                swerve.drive(0.0, 1.0, 0.0, false, false, false);
            }, () -> { swerve.stop(); }),
            new TimedCommand(2.5, () -> {
                swerve.drive(0.0, 0.0, 1.0, false, false, false);
            }, () -> { swerve.stop(); }),
            new RunIntake(cuber, 2.5),
            new Shoot(cuber, cuber, swerve, 2.5, lights),
            new RunIntake(coner, 2.5),
            new Shoot(coner, coner, swerve, 2.5, lights)
        );
    }
}
