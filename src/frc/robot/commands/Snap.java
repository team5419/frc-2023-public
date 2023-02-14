package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class Snap extends SequentialCommandGroup {
    public Snap(Swerve drivetrain, Vision vision, XboxController driver) {
        double deg = ((int)(((double)((driver.getPOV() + 45) % 360)) / 90)) * 90;
        addCommands(
            new RamseteSwerve(drivetrain, vision, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(deg)), true, true)
        );
    }
}