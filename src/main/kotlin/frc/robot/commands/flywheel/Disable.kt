package frc.robot.commands.flywheel

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.subsystems.FlywheelSubsystem

open class Disable : InstantCommand( { FlywheelSubsystem.enabled = false
    FlywheelSubsystem.target = 0.0} as Runnable)