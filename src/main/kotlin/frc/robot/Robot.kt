package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.networktables.NetworkTableInstance

val tab: ShuffleboardTab = Shuffleboard.getTab("Master")

class Robot : TimedRobot() {
  private val robotContainer = RobotContainer(tab);
  private var autoCommand = robotContainer.getAutonomousCommand();

  override fun robotInit() {
  }

  override fun robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  override fun disabledInit() {
  }

  override fun disabledPeriodic() {}

  override fun autonomousInit() {
    autoCommand = robotContainer.getAutonomousCommand();
    autoCommand.schedule();
  }

  override fun autonomousPeriodic() {}

  override fun teleopInit() {
    autoCommand.cancel();
  }

  override fun teleopPeriodic() {}

  override fun testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  override fun testPeriodic() {}
}
