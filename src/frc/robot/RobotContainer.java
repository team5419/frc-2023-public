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

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {

	public Cuber cuber;
	public Vision vision;
	//public Sensors sensors;
	private SendableChooser<Supplier<SequentialCommandGroup>> autoSelector;
	public SequentialCommandGroup currentAuto;
	//private Drivetrain drivetrain;
	public Swerve swerve;
	private XboxController driver;
	private XboxController codriver;
	public Coner coner;
	private boolean madeHub;
	public Lights lights;
	public Elevator elevator;
	private SimpleWidget reloader;
	public RobotContainer() {
		ShuffleboardTab tab = Shuffleboard.getTab("Auto");
		// PowerDistribution dist = new PowerDistribution(1, ModuleType.kRev);
		// 	dist.setSwitchableChannel(true);
		// 	dist.close();
		currentAuto = null;
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
		lights = new Lights();
	
		autoSelector = new SendableChooser<Supplier<SequentialCommandGroup>>();
		tab.add("Auto selector", autoSelector).withSize(2, 1).withPosition(0, 0);
		autoSelector.setDefaultOption("Baseline", () -> new Baseline());
		autoSelector.addOption("Preload only", () -> new PreloadOnly(swerve, vision, coner, cuber, lights));
		autoSelector.addOption("Charge Station Only", () -> new ChargeOnly(swerve, vision, coner, cuber, lights));
		autoSelector.addOption("Charge Station Back", () -> new ChargeStationBack(swerve, vision, coner, cuber, lights));
		autoSelector.addOption("GOAT", () -> new ConeDoubleCube().setupWith(this));
		ChoicedAuto.handleReqs(ConeDoubleCube.requirements);
		autoSelector.addOption("Three Cube", () -> new ThreeCube().setupWith(this));
		ChoicedAuto.handleReqs(ThreeCube.requirements);
		autoSelector.addOption("Two Cube + Balance", () -> new TwoCubeBalance().setupWith(this));
		ChoicedAuto.handleReqs(TwoCubeBalance.requirements);

		configureButtonBindings();
		setDefaults();
		reloader = tab.add("Reload", false)
            .withSize(2, 1)
			.withPosition(2, 0)
            .withWidget(BuiltInWidgets.kToggleSwitch);
		tab.addBoolean("Has auto", () -> currentAuto != null)
			.withSize(1, 1)
			.withPosition(4, 0);

		
	}
	public void loadAuto() {
		if(reloader.getEntry().getBoolean(false)) {
			if(currentAuto == null) {
				currentAuto = autoSelector.getSelected().get();
			}
		} else {
			currentAuto = null;
		}
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
		Trigger xButtonDriver = new Trigger(driver::getXButton);
		xButtonDriver.onTrue(Commands.runOnce(() -> {
			double mid = Elevator.heights.get(TargetHeights.MID);
			double high = Elevator.heights.get(TargetHeights.HIGH);
			if(elevator.state == mid) {
				elevator.state = high;
			} else if(elevator.state == high) {
				elevator.state = Elevator.down;
			} else {
				elevator.state = mid;
			}
		}));
		Trigger rightTrigger = new Trigger(() -> driver.getRightTriggerAxis() > SwerveDriveConstants.triggerDeadband);
		Trigger leftTrigger = new Trigger(() -> driver.getLeftTriggerAxis() > SwerveDriveConstants.triggerDeadband);
		Trigger leftBumper = new Trigger(driver::getLeftBumper);
		Trigger rightBumper = new Trigger(() -> driver.getRightBumper());
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
		leftTriggerCodriver.whileTrue(new AutoBalance(swerve, lights, vision, -1));
		rightTriggerCodriver.whileTrue(Commands.runEnd(cuber::runIntake, () -> cuber.stop(TargetHeights.LOW), cuber.subsystem()));
		aButtonCodriver.whileTrue(Commands.run(() -> coner.shoot(TargetHeights.INTAKE)));
		aButtonCodriver.onFalse(new SlightOutake(coner));
		yButtonCodriver.onTrue(new ResetGyro(swerve, 180.0));
		leftBumperCodriver.onTrue(Commands.runOnce(() -> swerve.autoShoot = !swerve.autoShoot));
	}

	public void useVision(boolean use) {
		swerve.usingVision = use;
		if(use) {
			swerve.foundPosition = false;
		}
	}

	private void setDefaults() {
		swerve.setDefaultCommand(new SwerveDrive(swerve, driver, codriver, cuber, vision));
	}
}