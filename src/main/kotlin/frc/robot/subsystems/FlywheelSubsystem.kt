package frc.robot.subsystems

import com.revrobotics.CANEncoder
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel.MotorType
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.FlywheelConstants
import frc.robot.util.constrain

object FlywheelSubsystem : SubsystemBase() {
  private val leftMotor = CANSparkMax(FlywheelConstants.LEFT_MOTOR_ID, MotorType.kBrushless)
  private val rightMotor = CANSparkMax(FlywheelConstants.RIGHT_MOTOR_ID, MotorType.kBrushless)

  private val hoodMotor = CANSparkMax(FlywheelConstants.HOOD_MOTOR_ID, MotorType.kBrushed)

  private val leftEncoder = CANEncoder(leftMotor)
  private val rightEncoder = CANEncoder(rightMotor)

  var enabled = false

  // Volt over RPM
  private var kP = 0.0
  private var kI = 0.0
  private var kD = 0.0
  private var kV = 0.0

  var target = 0.0 // RPM
    set(value) { field = constrain(value = value, max = FlywheelConstants.MAX_RPM) }

  private var lastError = 0.0
  private var totalError = 0.0

  @Override
  override fun periodic() {
    if (!enabled) {
      target = 0.0
      return
    }
    val error = target - getRPM()
    totalError += error
    val output = constrain(
        value = kV * target + kP * error + kI * totalError + kD * (error - lastError),
        max = 12.0
    )
    leftMotor.setVoltage(output)
    rightMotor.setVoltage(output)
    lastError = error
  }

  fun getRPM(): Double {
    return (leftEncoder.velocity + rightEncoder.velocity) / 2
  }
}