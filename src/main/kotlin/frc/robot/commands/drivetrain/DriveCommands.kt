package frc.robot.commands.drivetrain

import edu.wpi.first.wpilibj2.command.RunCommand
import frc.robot.subsystems.DrivetrainSubsystem

open class ArcadeDrive(throttle: Double, twist: Double, squareInputs: Boolean = true) : RunCommand(
    {DrivetrainSubsystem.arcadeDrive(throttle, twist, squareInputs)} as Runnable,
    DrivetrainSubsystem
)

open class SniperArcadeDrive(throttle: Double, twist: Double, throttleFactor: Double = 0.3,
                        twistFactor: Double = 0.3, squareInputs: Boolean = true) : ArcadeDrive(
    throttle = throttle * throttleFactor,
    twist = twist * twistFactor,
    squareInputs = squareInputs
)

open class OpenLoopStraightDrive(throttle: Double, squareInputs: Boolean = true) : ArcadeDrive(
    throttle = throttle,
    twist = 0.0,
    squareInputs = squareInputs
)