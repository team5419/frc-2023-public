package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.AprilTags;
public class AlignShoot extends SequentialCommandGroup {
    public AlignShoot(Swerve drivetrain, Vision vision, Intake intake, XboxController controller, boolean useClosest) {
        int num = drivetrain.currentNum;
        int height = drivetrain.currentHeight;
        Pose2d pose = drivetrain.pose();
        double angle = drivetrain.angle();
        if(useClosest) {
            double currentY = pose.getY();
            int closestNum = 0;
            for(int i = 1; i < AprilTags.yPositions.length; i++) {
                if(AprilTags.yPositions[i] > currentY) {
                    closestNum = (Math.abs(AprilTags.yPositions[i - 1] - currentY) < Math.abs(AprilTags.yPositions[i] - currentY)) ? (i - 1) : i;
                    break;
                }
            }
            num = closestNum;
        }
        boolean isCone = (num - 1) % 3 != 0; 
        double targetRotation = Math.round((angle  - (isCone ? AprilTags.coneRotation : AprilTags.cubeRotation)) / 360.0);
        targetRotation *= 360.0;
        targetRotation += isCone ? AprilTags.coneRotation : AprilTags.cubeRotation;
        Rotation2d converted = Rotation2d.fromDegrees(targetRotation);
        double effectiveX = pose.getX();
        double targetX = isCone ? AprilTags.coneDists[height] : AprilTags.cubeDists[height];
        double targetY = AprilTags.yPositions[num];
        if(effectiveX > AprilTags.xEndOfChargingStation) { // if hasn't made it to end of charging station, drive straight to end of station to avoid collision
            addCommands(new RamseteSwerve(drivetrain, new Pose2d(AprilTags.xEndOfChargingStation, pose.getY(), converted)));
        } else if(effectiveX < AprilTags.xPosBeforeBarriers) {
            addCommands(new RamseteSwerve(drivetrain, new Pose2d(AprilTags.xPosBeforeBarriers, pose.getY(), converted)));
        }
        addCommands(new RamseteSwerve(drivetrain, new Pose2d(Math.max(AprilTags.xPosBeforeBarriers, targetX), targetY, converted)));
        if(targetX < AprilTags.xPosBeforeBarriers) { // if not far enough, drive straight forward to goal
            addCommands(new RamseteSwerve(drivetrain, new Pose2d(targetX, targetY, converted)));
        }
        if(isCone) {
            addCommands(new AutoAlign(drivetrain, vision)); // if cone, use vision to fine-tune (might not be necessary)
        }
        // add a command to shoot
        addCommands(new RunIntake(intake, -1.0, 5.0)); // run intake full speed backwards for 5 seconds
    }
}