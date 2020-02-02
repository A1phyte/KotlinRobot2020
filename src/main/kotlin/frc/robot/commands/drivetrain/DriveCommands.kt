package frc.robot.commands.drivetrain

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.RunCommand
import frc.robot.Constants.DrivetrainConstants
import frc.robot.subsystems.DrivetrainSubsystem
import kotlin.math.abs

open class ClosedLoopArcadeDrive(private val joystick: Joystick, private val squareInputs: Boolean) : CommandBase() {
  init {
    addRequirements(DrivetrainSubsystem)
  }

  override fun execute() {
    var throttle = -joystick.y
    var twist = -joystick.z

    if (squareInputs) {
      throttle *= abs(throttle)
      twist *= abs(twist)
    }

    val targetSpeeds = DrivetrainSubsystem.kinematics.toWheelSpeeds(ChassisSpeeds(
        throttle * DrivetrainConstants.MAX_SPEED,
        0.0,
        twist * DrivetrainConstants.MAX_ANGULAR_VELOCITY
        ))

    var leftSpeed = targetSpeeds.leftMetersPerSecond / DrivetrainConstants.DISTANCE_PER_REVOLUTION
    var rightSpeed = targetSpeeds.rightMetersPerSecond / DrivetrainConstants.DISTANCE_PER_REVOLUTION

    if (leftSpeed > DrivetrainConstants.MAX_SPEED) {
      rightSpeed *= (DrivetrainConstants.MAX_SPEED / leftSpeed)
      leftSpeed = DrivetrainConstants.MAX_SPEED
    } else if (rightSpeed > DrivetrainConstants.MAX_SPEED) {
      leftSpeed *= (DrivetrainConstants.MAX_SPEED / rightSpeed)
      rightSpeed = DrivetrainConstants.MAX_SPEED
    }

    DrivetrainSubsystem.closedLoopDrive(leftSpeed, rightSpeed);
  }

  override fun isFinished(): Boolean {
    return DrivetrainSubsystem.closedLoopAccumError >= DrivetrainConstants.SM_I_LIMIT
  }

  override fun end(interrupted: Boolean) {
    if (!interrupted) {
      DrivetrainSubsystem.defaultCommand = OpenLoopArcadeDrive(joystick, squareInputs)
    }
  }
}

open class OpenLoopArcadeDrive(private val joystick: Joystick, private val squareInputs: Boolean) : CommandBase() {
  init {
    addRequirements(DrivetrainSubsystem)
  }

  override fun execute() {
    var throttle = -joystick.y
    var twist = -joystick.z

    if (squareInputs) {
      throttle *= abs(throttle)
      twist *= abs(twist)
    }

    val targetSpeeds = DrivetrainSubsystem.kinematics.toWheelSpeeds(ChassisSpeeds(
        throttle * DrivetrainConstants.MAX_SPEED,
        0.0,
        twist * DrivetrainConstants.MAX_ANGULAR_VELOCITY
    ))

    var leftSpeed = targetSpeeds.leftMetersPerSecond / DrivetrainConstants.DISTANCE_PER_REVOLUTION
    var rightSpeed = targetSpeeds.rightMetersPerSecond / DrivetrainConstants.DISTANCE_PER_REVOLUTION

    if (leftSpeed > DrivetrainConstants.MAX_SPEED) {
      rightSpeed *= (DrivetrainConstants.MAX_SPEED / leftSpeed)
      leftSpeed = DrivetrainConstants.MAX_SPEED
    } else if (rightSpeed > DrivetrainConstants.MAX_SPEED) {
      leftSpeed *= (DrivetrainConstants.MAX_SPEED / rightSpeed)
      rightSpeed = DrivetrainConstants.MAX_SPEED
    }

    DrivetrainSubsystem.openLoopDrive(leftSpeed, rightSpeed);
  }
}