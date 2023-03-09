package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoGetCube;
import frc.robot.commands.RamseteSwerve;
import frc.robot.commands.Shoot;
import frc.robot.commands.UseVision;
import frc.robot.subsystems.GenericShootIntake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.TargetHeights;
import frc.robot.classes.RamseteOptions;
import frc.robot.Constants.AutoConstants;

public class AutoBuilder extends SequentialCommandGroup {
    private Translation2d createTranslation(Translation2d original, double multiplier) {
        if(multiplier == 1.0) {
            return original;
        }
        return new Translation2d(original.getX(), original.getY() * multiplier);
    }
    public AutoBuilder(String input, Swerve drivetrain, Vision vision, GenericShootIntake coneShooter, GenericShootIntake cubeShooter, Lights lights) {
        boolean shor = false;
        int start = 0;
        if(input.startsWith("S")) {
            shor = true;
            start = 1;
        }
        double multiplier = shor ? -1.0 : 1.0;
        boolean gettingCube = input.contains("1");
        addCommands(new UseVision(drivetrain, false), // disable vision
        Commands.runOnce(() -> { // reset position and drop cone intake
            drivetrain.resetGyro(180.0);
            drivetrain.usingCones = false; // just to set lights to purple :)))
            coneShooter.setup(TargetHeights.INTAKE);
            drivetrain.currentHeight = 1;
            if(gettingCube) {
                cubeShooter.setup(TargetHeights.INTAKE);
            }
        }));
        boolean hasGoneToTwo = false;
        for(int i = start; i < input.length(); i++) {
            char cha = input.charAt(i);
            switch(cha) {
                case 'A':
                    addCommands(new AutoAlign(drivetrain, coneShooter, vision, coneShooter.getLimelightDistance(TargetHeights.heights[drivetrain.currentHeight]), drivetrain.currentHeight, lights, 1.5));
                    break;
                case 'P':
                    addCommands(new Shoot(coneShooter, coneShooter, drivetrain, 1.0, lights), // shoot pre-load cone and retract cone intake
                    Commands.runOnce(() -> { // drop cube intake and start spinning intake
                        drivetrain.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0)));
                    }));
                    break;
                case '1':
                    addCommands(new RamseteSwerve(drivetrain, vision, new Pose2d(createTranslation(new Translation2d(3.8, 0.35), multiplier), Rotation2d.fromDegrees(180.0)), new RamseteOptions(true, false, false, 4.0, -1, shor ? -1.0 : 2.0, 0.0)),
                    new AutoGetCube(drivetrain, cubeShooter, vision, createTranslation(AutoConstants.firstCube, multiplier), createTranslation(shor ? AutoConstants.firstShotShortSide : AutoConstants.firstShot, multiplier), shor ? 3 : 1, lights, !shor));
                    break;
                case '2':
                    hasGoneToTwo = true;
                    addCommands(new RamseteSwerve(drivetrain, vision, new Pose2d(createTranslation(new Translation2d(3.4, 1.95), multiplier), Rotation2d.fromDegrees(180.0)), new RamseteOptions(false, 6.0)),
                    new AutoGetCube(drivetrain, cubeShooter, vision, createTranslation(AutoConstants.secondCube, multiplier), createTranslation(AutoConstants.secondShot, multiplier), 2, lights, false));
                    break;
                case 'c':
                    addCommands(new RamseteSwerve(drivetrain, vision, new Pose2d(createTranslation(new Translation2d(1.8, 0.0), multiplier), Rotation2d.fromDegrees(180.0)), new RamseteOptions(true,  false, false, 4.0, -1, 2.0, 0.0)));
                    break;
                case 'C':
                    if(!hasGoneToTwo) {
                        addCommands(new RamseteSwerve(drivetrain, vision, new Pose2d(createTranslation(new Translation2d(3.4, 1.95), multiplier), Rotation2d.fromDegrees(180.0)), new RamseteOptions(false, 6.0)));
                        hasGoneToTwo = true;
                    }
                    addCommands(Commands.runOnce(() -> {
                        cubeShooter.stop(TargetHeights.INTAKE);
                    }),
                    new RamseteSwerve(drivetrain, vision, new Pose2d(createTranslation(AutoConstants.preBalancePosition, multiplier), new Rotation2d(0.0)), new RamseteOptions(false, 4.0)));
                    break;
                case 'K':
                    addCommands(new AutoBalance(drivetrain, lights));
                default:
                    System.out.println("Unrecognized character: " + cha);
                    break;
            }
        }
    }
}
