package frc.robot;

import frc.robot.imp.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Robot extends TimedRobot {
  private ShuffleboardTab tab;
  private RobotContainer robotContainer;
  private Command autoCommand;
  public Robot() {
    tab = Shuffleboard.getTab("Master");
    robotContainer = new RobotContainer();
    autoCommand = null;
  }

  public void robotInit() {
  }

  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  public void disabledInit() {
    robotContainer.swerve.stop();
    robotContainer.useVision(true);
    robotContainer.vision.on();
    robotContainer.lights.off();
  }

  public void disabledPeriodic() {
    robotContainer.loadAuto();
  }

  public void autonomousInit() {
    robotContainer.vision.off();
    autoCommand = robotContainer.currentAuto;
    autoCommand.schedule();
  }

  public void autonomousPeriodic() {}

  public void teleopInit() {
    robotContainer.vision.off();
    robotContainer.useVision(true);
    robotContainer.lights.off(robotContainer.swerve);
    if(autoCommand != null) {
      autoCommand.cancel();
    }
  }

  public void teleopPeriodic() {}

  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  public void testPeriodic() {}
}
