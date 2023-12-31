package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.RamseteOptions;
import frc.robot.commands.driving.RamseteSwerve;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class Snap extends CommandBase {
    Swerve drivetrain;
    Vision vision;
    XboxController driver;
    int mod;
    public Snap(Swerve drivetrain, Vision vision, XboxController driver, int mod) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.driver = driver;
        this.mod = mod;
    }

    public void initialize() {
        double deg = (((this.driver.getPOV() + (this.drivetrain.usingCones ? 0 : 180) + 45) % 360) / 90) * 90;
        RamseteSwerve ramsete = new RamseteSwerve(this.drivetrain, this.vision, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(deg)), new RamseteOptions(true, true));
        ramsete.schedule();
    }   

    public boolean isFinished() {
        return true;
    }
}