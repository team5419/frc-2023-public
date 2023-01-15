package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class goToPosition extends CommandBase {
    private XboxController joystick;
    private Pose2d desiredPosition;
    private Vision vis;
    private Swerve swerve;
    private Rotation2d rotation;
    public goToPosition(ShuffleboardTab tab){
        joystick = new XboxController(1);
        rotation = new Rotation2d(0);
        desiredPosition = new Pose2d(14,2,rotation); // the desired position for the robot to move to
        vis = new Vision(tab);
        swerve = new Swerve(vis);
    }
    
    
    public void initialize() {
        
    }
    public void execute() {
        swerve.drive(swerve.pose().getX() - desiredPosition.getX(), swerve.pose().getY() - desiredPosition.getY(),0);
        //swerves the robot over to the desired position
    }
    public boolean isFinished() {
       return swerve.pose() == desiredPosition;//finishes command when at desired position
    }
    public void end(boolean interrupted) {
       swerve.brake();// brakes the swerve when command is finished
    }
}
