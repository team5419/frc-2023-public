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
    private double getEffective(Vision.Team team, double value) {
        return team == Vision.Team.RED ? (AprilTags.fieldLength - value) : value;
    }
    public AlignShoot(Swerve drivetrain, Vision vision, Intake intake, XboxController controller, boolean useClosest) {
        Vision.Team team = vision.team;
        if(team == Vision.Team.NONE) {
            return;
        }
        int num = drivetrain.currentNum;
        int height = drivetrain.currentHeight;
        Pose2d pose = drivetrain.pose();
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
        } else {
            if(team == Vision.Team.BLUE) {
                num = AprilTags.yPositions.length - num - 1;
            }
        }
        boolean isCone = (num - 1) % 3 != 0; 
        double effectiveX = getEffective(team, pose.getX());
        double targetX = isCone ? AprilTags.coneDists[height] : AprilTags.cubeDists[height];
        double targetY = AprilTags.yPositions[num];
        if(effectiveX > AprilTags.xEndOfChargingStation) { // if hasn't made it to end of charging station, drive straight to end of station to avoid collision
            addCommands(new RamseteSwerve(drivetrain, new Pose2d(getEffective(team, AprilTags.xEndOfChargingStation), pose.getY(), new Rotation2d(0.0))));
        }
        double beforeBarrierDist = getEffective(team, AprilTags.xPosBeforeBarriers); // drive diagonally to target, but don't hit barriers between goals
        addCommands(new RamseteSwerve(drivetrain, new Pose2d(getEffective(team, Math.max(beforeBarrierDist, targetX)), targetY, new Rotation2d(0.0))));
        if(targetX < beforeBarrierDist) { // if not far enough, drive straight forward to goal
            addCommands(new RamseteSwerve(drivetrain, new Pose2d(getEffective(team, targetX), targetY, new Rotation2d(0.0))));
        }
        if(isCone) {
            addCommands(new AutoAlign(drivetrain, vision)); // if cone, use vision to fine-tune (might not be necessary)
        }
        // add a command to shoot
        addCommands(new RunIntake(intake, -1.0, 5.0)); // run intake full speed backwards for 5 seconds
    }
}