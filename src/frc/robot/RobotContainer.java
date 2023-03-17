package frc.robot;
import frc.robot.Constants.ConerTypes;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.TargetHeights;
import frc.robot.auto.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.test.TesterSubsystem;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {
	private final ConerTypes CONER_TYPE = ConerTypes.LOW_CONER; // THIS IS HOW YOU CHANGE WHICH CONE SHOOTER TO USE!!!!!

	private Cuber cuber;
	public Vision vision;
	public Sensors sensors;
	private SendableChooser<SequentialCommandGroup> autoSelector;
	//private Drivetrain drivetrain;
	public Swerve swerve;
	private XboxController driver;
	private XboxController codriver;
	private Coner coner;
	private EverybotArm arm;
	private boolean madeHub;
	public Lights lights;
	private GenericEntry autoEntry;

	public RobotContainer(ShuffleboardTab tab) {
		
		driver = new XboxController(0);
		codriver = new XboxController(1);
		vision = new Vision(tab, true, true);
		sensors = new Sensors(tab);
		swerve = new Swerve(vision, true); /* CHOOSE ONE!!! */
		arm = null; 
		madeHub = false;
		coner = new Coner(true, false);
		generateHub();
		cuber = new Cuber(true);
		
	
		
		//drivetrain = new Drivetrain(); /* ^^^ */
		lights = new Lights();
	
		autoSelector = new SendableChooser<SequentialCommandGroup>();
		tab.add("Auto selector", autoSelector).withSize(2, 1).withPosition(0, 0);
		autoSelector.setDefaultOption("Baseline", new Baseline());
		autoSelector.addOption("Two Cube Balance", new TwoCubeBalance(swerve, vision, coner, cuber, lights));
		autoSelector.addOption("Systems Check", new SystemsCheck(swerve, cuber, coner, lights));

		autoSelector.addOption("Preload only", new PreloadOnly(swerve, vision, coner, cuber, false, lights));

		autoSelector.addOption("First Cube Only", new FirstCubeOnly(swerve, vision, coner, cuber, false, lights));
		autoSelector.addOption("First Cube Only, Short Side", new FirstCubeOnly(swerve, vision, coner, cuber, true, lights));
		autoSelector.addOption("Charge Station Back", new ChargeStationBack(swerve, vision, coner, cuber, false, lights));
		autoSelector.addOption("Charge Station Only", new ChargeOnly(swerve, vision, coner, cuber, false, lights));
		configureButtonBindings();
		setDefaults();

		autoEntry = tab.add("Auto input", "")
            .withPosition(0, 1)
            .withSize(2, 1)
            .getEntry();
	}

	private EverybotArm generateArm() {
		if(arm == null) {
			arm = new EverybotArm(true);
		}
		return arm;
	}

	public void setArmState(boolean state) {
		if(arm != null) {
			arm.enabled = state;
		}
	}

	private void generateHub() {
		if(!madeHub) {
			Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
			compressor.enableDigital();
			madeHub = true;
		}
		
		// if(hub == null) {
		// 	PowerDistribution dist = new PowerDistribution(1, ModuleType.kRev);
		// 	dist.setSwitchableChannel(true);
		// 	dist.close();
		// 	hub = new PneumaticHub();
			
		// 	Compressor compressor = hub.makeCompressor();
		// 	compressor.enableDigital();
		// }
		// return hub;
	}
  
	private void configureButtonBindings() {
		Trigger aButtonDriver = new Trigger(() -> driver.getAButton());
		Trigger bButtonDriver = new Trigger(() -> driver.getBButton());
		Trigger yButtonDriver = new Trigger(() -> driver.getYButton());
		Trigger rightTrigger = new Trigger(() -> driver.getRightTriggerAxis() > SwerveDriveConstants.triggerDeadband);
		Trigger leftTrigger = new Trigger(() -> driver.getLeftTriggerAxis() > SwerveDriveConstants.triggerDeadband);
		Trigger leftBumper = new Trigger(() -> driver.getLeftBumper());
		Trigger rightBumper = new Trigger(() -> driver.getRightBumper() || driver.getXButton());
		Trigger dpad = new Trigger(() -> driver.getPOV() != -1);

		Trigger aButtonCodriver = new Trigger(() -> codriver.getAButton());
		Trigger bButtonCodriver = new Trigger(() -> codriver.getBButton());
		Trigger xButtonCodriver = new Trigger(() -> codriver.getXButton());
		Trigger yButtonCodriver = new Trigger(() -> codriver.getYButton());
		Trigger leftBumperCodriver = new Trigger(() -> codriver.getLeftBumper());
		Trigger rightBumperCodriver = new Trigger(() -> codriver.getRightBumper());
		Trigger rightTriggerCodriver = new Trigger(() -> codriver.getRightTriggerAxis() > SwerveDriveConstants.triggerDeadband);
		Trigger leftTriggerCodriver = new Trigger(() -> codriver.getLeftTriggerAxis() > SwerveDriveConstants.triggerDeadband);
		
		 Trigger leftStickPressCodriver = new Trigger(() -> codriver.getLeftStickButton());
// leftStickPressCodriver.onTrue(new AutoAlign(swerve, coner, vision, 1.0, lights, 1.0 ));
		leftStickPressCodriver.onTrue(new ResetGyro(swerve, 180.0));
		
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
		rightBumperCodriver.onTrue(Commands.runOnce(() -> {
			swerve.usingCones = !swerve.usingCones;
			lights.off(swerve);
		}));
		aButtonDriver.whileTrue(new Prep(coner, cuber, swerve, null));
		bButtonDriver.whileTrue(new Shoot(coner, cuber, swerve, lights));
		leftTriggerCodriver.whileTrue(new TeleopBalance(swerve, lights, driver));
		yButtonDriver.onTrue(new ResetGyro(swerve));
		leftTrigger.whileTrue(new RunIntake(coner));
		rightTrigger.whileTrue(new RunIntake(cuber));
		rightTriggerCodriver.whileTrue(Commands.runEnd(() -> cuber.shoot(TargetHeights.INTAKE), () -> cuber.stop(TargetHeights.LOW), cuber.subsystem()));
		aButtonCodriver.onTrue(new ChangeSystemOffset(1, -0.0025, cuber, coner, swerve));
		yButtonCodriver.onTrue(new ChangeSystemOffset(1, 0.0025, cuber, coner, swerve));
		xButtonCodriver.onTrue(new ChangeSystemOffset(0, 0.0025, cuber, coner, swerve));
		bButtonCodriver.onTrue(new ChangeSystemOffset(0, -0.0025, cuber, coner, swerve));
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
		if(arm != null) {
			arm.setDefaultCommand(new ManualMoveArm(arm, codriver));
		}
	}
}