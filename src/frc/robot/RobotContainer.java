package frc.robot;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.TargetHeights;
import frc.robot.auto.*;
import frc.robot.commands.*;
import frc.robot.commands.driving.AutoBalance;
import frc.robot.commands.driving.SpecialRamseteTurn;
import frc.robot.commands.driving.SwerveDrive;
// import frc.robot.commands.driving.TeleopBalance;
import frc.robot.commands.shooting.Prep;
import frc.robot.commands.shooting.RunIntake;
import frc.robot.commands.shooting.Shoot;
import frc.robot.commands.shooting.SlightOutake;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {

	private Cuber cuber;
	public Vision vision;
	//public Sensors sensors;
	private SendableChooser<SequentialCommandGroup> autoSelector;
	//private Drivetrain drivetrain;
	public Swerve swerve;
	private XboxController driver;
	private XboxController codriver;
	private Coner coner;
	private boolean madeHub;
	public Lights lights;
	private Elevator elevator;

	public RobotContainer(ShuffleboardTab tab) {
		// PowerDistribution dist = new PowerDistribution(1, ModuleType.kRev);
		// 	dist.setSwitchableChannel(true);
		// 	dist.close();
		
		driver = new XboxController(0);
		codriver = new XboxController(1);
		vision = new Vision(tab, true, true);
		//sensors = new Sensors(tab);
		swerve = new Swerve(vision, true); /* CHOOSE ONE!!! */
		madeHub = false;
		elevator = new Elevator();
		coner = new Coner(elevator, true, false);
		generateHub();
		cuber = new Cuber(true);
		
	
		
		//drivetrain = new Drivetrain(); /* ^^^ */
		lights = new Lights();
	
		autoSelector = new SendableChooser<SequentialCommandGroup>();
		tab.add("Auto selector", autoSelector).withSize(2, 1).withPosition(0, 0);
		autoSelector.setDefaultOption("Baseline", new Baseline());
		autoSelector.addOption("3 Cube Balance R", new TwoCubeBalance(swerve, vision, coner, cuber, lights, true, true));
		autoSelector.addOption("3 Cube R", new TwoCubeBalance(swerve, vision, coner, cuber, lights, false, true));
		autoSelector.addOption("4 Cube Balance R", new ThreeCube(swerve, vision, coner, cuber, lights, true, true));
		autoSelector.addOption("4 Cube R", new ThreeCube(swerve, vision, coner, cuber, lights, false, true));
		autoSelector.addOption("3 Cube Balance B", new TwoCubeBalance(swerve, vision, coner, cuber, lights, true, false));
		autoSelector.addOption("3 Cube B", new TwoCubeBalance(swerve, vision, coner, cuber, lights, false, false));
		autoSelector.addOption("4 Cube Balance B", new ThreeCube(swerve, vision, coner, cuber, lights, true, false));
		autoSelector.addOption("4 Cube B", new ThreeCube(swerve, vision, coner, cuber, lights, false, false));
		autoSelector.addOption("Preload only", new PreloadOnly(swerve, vision, coner, cuber, false, lights));
		autoSelector.addOption("Charge Station Only", new ChargeOnly(swerve, vision, coner, cuber, false, lights));
		autoSelector.addOption("Test Using Waypoints 1", new TestWaypointing(swerve));
		autoSelector.addOption("Test Using Waypoints 2", new TestWaypointing2(swerve));
		autoSelector.addOption("Test Using Waypoints 3", new TestWaypointing3(swerve, vision));
		autoSelector.addOption("new auto", new ConeDoubleCube(swerve, vision, coner, cuber, lights, true, false));
		configureButtonBindings();
		setDefaults();
	}
	private void generateHub() {
		if(!madeHub) {
			Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
			compressor.enableDigital();
			compressor.close();
			madeHub = true;
		}
	}
  
	private void configureButtonBindings() {
		// driver controls
		Trigger aButtonDriver = new Trigger(driver::getAButton);
		Trigger bButtonDriver = new Trigger(driver::getBButton);
		Trigger yButtonDriver = new Trigger(driver::getYButton);
		Trigger rightTrigger = new Trigger(() -> driver.getRightTriggerAxis() > SwerveDriveConstants.triggerDeadband);
		Trigger leftTrigger = new Trigger(() -> driver.getLeftTriggerAxis() > SwerveDriveConstants.triggerDeadband);
		Trigger leftBumper = new Trigger(driver::getLeftBumper);
		Trigger rightBumper = new Trigger(() -> driver.getRightBumper() || driver.getXButton());
		Trigger dpad = new Trigger(() -> driver.getPOV() != -1);
		Trigger alignControllerOff = new Trigger(() -> swerve.isAligning == Swerve.AlignState.CONTROLLERON && (driver.getLeftX() < SwerveDriveConstants.controllerDeadband && 
				driver.getLeftY() < SwerveDriveConstants.controllerDeadband &&
				driver.getRightX() < SwerveDriveConstants.controllerDeadband &&
				driver.getRightY() < SwerveDriveConstants.controllerDeadband));
		Trigger endAlign = new Trigger(() -> swerve.isAligning == Swerve.AlignState.CONTROLLEROFF && (driver.getLeftX() > SwerveDriveConstants.controllerDeadband || 
									driver.getLeftY() > SwerveDriveConstants.controllerDeadband || 
									driver.getRightX() > SwerveDriveConstants.controllerDeadband || 
									driver.getRightY() > SwerveDriveConstants.controllerDeadband));
		endAlign.onTrue(Commands.runOnce(() -> {
			swerve.isAligning = Swerve.AlignState.NOT; // cancel align
		}, swerve));
		alignControllerOff.onTrue(Commands.runOnce(() -> {
			swerve.isAligning = Swerve.AlignState.CONTROLLEROFF;
		}));
		dpad.onTrue(new Snap(swerve, vision, driver, 4));
		leftBumper.whileTrue(Commands.runEnd(() -> { swerve.slowMode = true; }, () -> { swerve.slowMode = false; }));
		rightBumper.onTrue(new SpecialRamseteTurn(swerve, vision, driver, coner, cuber, lights));
		aButtonDriver.whileTrue(new Prep(coner, cuber, swerve, null));
		bButtonDriver.whileTrue(new Shoot(coner, cuber, swerve, lights));
		yButtonDriver.onTrue(new ResetGyro(swerve));
		leftTrigger.whileTrue(new RunIntake(coner));
		rightTrigger.whileTrue(new RunIntake(cuber));

		// codriver controls
		Trigger aButtonCodriver = new Trigger(codriver::getAButton);
		Trigger yButtonCodriver = new Trigger(codriver::getYButton);
		Trigger leftBumperCodriver = new Trigger(codriver::getLeftBumper);
		Trigger rightBumperCodriver = new Trigger(codriver::getRightBumper);
		Trigger rightTriggerCodriver = new Trigger(() -> codriver.getRightTriggerAxis() > SwerveDriveConstants.triggerDeadband);
		Trigger leftTriggerCodriver = new Trigger(() -> codriver.getLeftTriggerAxis() > SwerveDriveConstants.triggerDeadband);
		rightBumperCodriver.onTrue(Commands.runOnce(() -> {
			swerve.usingCones = !swerve.usingCones;
			lights.off(swerve);
		}));
		leftTriggerCodriver.whileTrue(new AutoBalance(swerve, lights));
		rightTriggerCodriver.whileTrue(Commands.runEnd(cuber::runIntake, () -> cuber.stop(TargetHeights.LOW), cuber.subsystem()));
		aButtonCodriver.whileTrue(Commands.run(() -> coner.shoot(TargetHeights.INTAKE)));
		aButtonCodriver.onFalse(new SlightOutake(coner));
		yButtonCodriver.onTrue(new ResetGyro(swerve, 180.0));
		leftBumperCodriver.onTrue(Commands.runOnce(() -> swerve.autoShoot = !swerve.autoShoot));
	}

	public Command getAutonomousCommand() {
		SequentialCommandGroup res = autoSelector.getSelected();
		return res;
	}

	public void useVision(boolean use) {
		swerve.usingVision = use;
		if(use) {
			swerve.foundPosition = false;
		}
	}

	private void setDefaults() {
		swerve.setDefaultCommand(new SwerveDrive(swerve, driver, codriver, cuber));
	}
}