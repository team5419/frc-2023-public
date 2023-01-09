package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
public class AlignShoot extends SequentialCommandGroup {
    public AlignShoot(Swerve drivetrain, Vision vision, Intake intake) {
        int num = drivetrain.currentNum;
        int height = drivetrain.currentHeight;
        boolean isCone = (num - 1) % 3 != 0; 
        addCommands(new AprilAlign(drivetrain, vision, num, height, isCone));
        if(isCone) {
            addCommands(new AutoAlign(drivetrain, vision));
        }
        // add a command to shoot
        addCommands(new RunIntake(intake, -1.0, 5.0)); // run intake full speed backwards for 5 seconds
    }
}