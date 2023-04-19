package frc.robot.auto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.TargetHeights;
import frc.robot.classes.RamseteOptions;
import frc.robot.commands.*;
import frc.robot.commands.driving.AutoBalance;
import frc.robot.commands.driving.RamseteSwerve;
import frc.robot.subsystems.Coner;
import frc.robot.subsystems.Cuber;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
// SP12CK or P12CK for 2 cubes, SP1CK or P1CK for one cube


public class ThreeCube extends ChoicedAuto { // basic routine for diff drive
    private static double shootX = 0.78;
    public SequentialCommandGroup setupWith(RobotContainer container) {
        boolean engage = getKey("engage");
        addCommands(
            new UseVision(container.swerve, false), // disable container.vision
            Commands.runOnce(() -> { // reset position and drop cone intake
                container.swerve.resetGyro(0.0);
                container.swerve.usingCones = false; // just to set container.lights to purple :)))
                container.swerve.currentHeight = 1;
                container.cuber.runPercentOutput(1, 0.7);
            }),
            new WaitCommand(0.5),
            Commands.runOnce(() -> {
                container.cuber.runPercentOutput(0, 1.0);
                container.cuber.runPercentOutput(1, 0.7);
            }),
            new WaitCommand(0.8),
            //new Shoot(container.cuber, container.cuber, container.swerve, 1.5, TargetHeights.FAR, container.lights), // shoot pre-load cone and retract cone intake
            Commands.runOnce(() -> { // drop cube intake and start spinning intake
                container.swerve.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
                container.cuber.state = container.cuber.down;
                container.cuber.runPercentOutput(0, -0.4);
                container.cuber.runPercentOutput(1, -0.65);
            }),
            //new MessyRamsete(container.swerve, container.vision, new Pose2d(new Translation2d(0.75, -0.18), Rotation2d.fromDegrees(0.0)), 4.0),
            new RamseteSwerve(container.swerve, container.vision, new Pose2d(new Translation2d(0.75, -0.18), Rotation2d.fromDegrees(0.0)), new RamseteOptions(true, false, false, 10.0, -1, -1.0, 0.0)),
            //new MessyRamsete(container.swerve, container.vision, new Pose2d(new Translation2d(1.5, -0.18), Rotation2d.fromDegrees(180.0)), 4.0), 
            new RamseteSwerve(container.swerve, container.vision, new Pose2d(new Translation2d(1.5, -0.13), Rotation2d.fromDegrees(180.0)), new RamseteOptions(true, false, false, 10.0, -1, -1.0, 0.0)),
            new AutoGetCube(container.swerve, container.cuber, container.coner, container.vision, new Translation2d(2.5, -0.13), new Translation2d(shootX, 0.75), -1, container.lights, false),
            Commands.runOnce(() -> { 
                container.cuber.state = container.cuber.down;
                container.cuber.runPercentOutput(0, -0.4);
                container.cuber.runPercentOutput(1, -0.65);
            }),
            //new MessyRamsete(container.swerve, container.vision, new Pose2d(new Translation2d(1.1, 0.75), Rotation2d.fromDegrees(0.0)), 4.0),
            new RamseteSwerve(container.swerve, container.vision, new Pose2d(new Translation2d(1.4, 0.75), Rotation2d.fromDegrees(0.0)), new RamseteOptions(true, false, false, 10.0, -1, -1.0, 0.0)),
            //new MessyRamsete(container.swerve, container.vision, new Pose2d(new Translation2d(1.65, 1.15), Rotation2d.fromDegrees(180.0)), 4.0),
            new RamseteSwerve(container.swerve, container.vision, new Pose2d(new Translation2d(1.8, 1.14), Rotation2d.fromDegrees(180.0)), new RamseteOptions(true, false, false, 10.0, -1, -1.0, 0.0)),
            new AutoGetCube(container.swerve, container.cuber, container.coner, container.vision, new Translation2d(2.5, 1.14), new Translation2d(shootX, 1.7), -1, container.lights, false),
            Commands.runOnce(() -> 
                container.cuber.shoot(TargetHeights.INTAKE)
            ),
            new RamseteSwerve(container.swerve, container.vision, new Pose2d(new Translation2d(1.4, 2.1), Rotation2d.fromDegrees(0.0)), new RamseteOptions(true, false, false, 5.0, -1, -1.0, 0.0))
             // drive back to second cube
        );
        if(engage) {
            addCommands(
                new AutoGetCube(container.swerve, container.cuber, container.coner, container.vision, new Translation2d(2.3, 2.1), new Translation2d(shootX, 1.87), -1, container.lights, false),
                new RamseteSwerve(container.swerve, container.vision, new Pose2d(new Translation2d(-0.4, 1.87), new Rotation2d(0.0)), new RamseteOptions(true,  false, false, 4.0, -1, -1.0, 0.0)),
            new AutoBalance(container.swerve, container.lights, container.vision, -1));
        } else {
            addCommands(
                new RamseteSwerve(container.swerve, container.vision, new Pose2d(new Translation2d(1.6, 2.1), Rotation2d.fromDegrees(180.0)), new RamseteOptions(true, false, false, 5.0, -1, -1.0, 0.0)),
                new RamseteSwerve(container.swerve, container.vision, new Pose2d(new Translation2d(2.3, 2.1), Rotation2d.fromDegrees(180.0)), new RamseteOptions(true, false, false, 2.0, -1, -1.0, 0.0)),
                new RamseteSwerve(container.swerve, container.vision, new Pose2d(new Translation2d(2.2, 2.1), new Rotation2d(0.0)), new RamseteOptions(true,  false, false, 1.0, -1, -1.0, 0.0)), 
            Commands.runOnce(() -> { // pull up intake
                container.cuber.stop(TargetHeights.INTAKE);
                //container.cuber.shoot(TargetHeights.INTAKE);
            }));
        }
        return this;
    }
    public static String[] requirements = { "engage" };
}