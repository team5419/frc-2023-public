package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.GenericShootIntake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class AlignSetup extends ParallelCommandGroup {
    public AlignSetup(GenericShootIntake coneShooter, GenericShootIntake cubeShooter, Swerve drivetrain, Vision vision, XboxController driver) {
        addCommands(
                new Prep(coneShooter, cubeShooter, drivetrain),
                new SpecialRamseteTurn(drivetrain, vision, driver, coneShooter, cubeShooter)
        );
    }
}
