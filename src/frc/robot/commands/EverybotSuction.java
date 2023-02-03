// package frc.robot.commands;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.EverybotArm;
// import edu.wpi.first.wpilibj.Timer;

// public class EverybotSuction extends CommandBase {
//     private boolean reverse;
//     private EverybotArm arm;
//     private Timer timer;

//     public EverybotSuction(EverybotArm _arm) {
//         this.arm = _arm;
//         this.timer = new Timer();
//         // addRequirements(arm);
//     }

//     public void initialize() {
//         this.timer.reset();
//         this.timer.start();
//         this.arm.open();
//         System.out.println("worky");
//     }  

//     public void execute() {
        
//     }

//     public boolean isFinished() {
//         return this.timer.get() > 0.1;
//     }

//     public void end(boolean interrupted) {
//         this.arm.close();
//         timer.stop();
//     }
// }
