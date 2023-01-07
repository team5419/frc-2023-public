package frc.robot.modules;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.modules.ModuleInfo;
interface ISwerveModule {
    fun getDrive(): Double;
    fun getTurn(): Rotation2d;
    fun getState(): SwerveModuleState;
    fun setDesiredState(desiredState: SwerveModuleState, preventTurn: Boolean, slow: Boolean, pid: Boolean);
    fun simulationPeriodic(dt: Double);
    fun resetEncoders();
    fun setBrakeMode(on: Boolean);
    fun test();
}