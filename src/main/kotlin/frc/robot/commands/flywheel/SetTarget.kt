package frc.robot.commands.flywheel

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.Constants.FlywheelConstants
import frc.robot.subsystems.FlywheelSubsystem
import frc.robot.util.constrain

open class SetTarget(target: Double) : InstantCommand({ FlywheelSubsystem.enabled = true
    FlywheelSubsystem.target = constrain(target, FlywheelConstants.MAX_RPM) } as Runnable)