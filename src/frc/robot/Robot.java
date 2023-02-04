package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
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
    robotContainer = new RobotContainer(tab);
    autoCommand = null;
  }

  public void robotInit() {
  }

  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  public void disabledInit() {
  }

  public void disabledPeriodic() {}

  public void autonomousInit() {
	robotContainer.setUpShooters();
    autoCommand = robotContainer.getAutonomousCommand();
    autoCommand.schedule();
  }

  public void autonomousPeriodic() {}

  public void teleopInit() {
    robotContainer.setDefaults(); // wait to call these functions until auto has started (A.K.A.)
    robotContainer.configureButtonBindings();
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
