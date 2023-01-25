//instructions: press Y to activate command, press B to deactivate it
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
    double dirY;
    double dirX;
    double speed;
    public goToPosition(ShuffleboardTab tab){
        joystick = new XboxController(1);
        rotation = new Rotation2d(0);
        desiredPosition = new Pose2d(14,2,rotation); // the desired position for the robot to move to
        vis = new Vision(tab);
        swerve = new Swerve(vis);
        speed = 100;
    }
    
    
    public void initialize() {
        dirX = desiredPosition.getX() - swerve.pose().getX();
        dirY = desiredPosition.getY() - swerve.pose().getY();
        // if (dirX > dirY){//makes sure the direction isnt greater when robot is further away
        //     dirX = 1;
        //     dirY = dirY/dirX;
        // }else{
        //     dirY = 1;
        //     dirX = dirX/dirY;
        // }
        // dirX *= speed;
        // dirY *= speed;
    }
    public void execute() {
        swerve.drive(dirX, dirY,0);
        //swerves the robot over to the desired position
    }
    public boolean isFinished() {
        if (joystick.getBButton()){
            return true;//stops command when B is pressed on controller
        }else{
            return swerve.pose() == desiredPosition;//finishes command when at desired position
        }
    }
    public void end(boolean interrupted) {
       swerve.brake();// brakes the swerve when command is finished
    }
}
