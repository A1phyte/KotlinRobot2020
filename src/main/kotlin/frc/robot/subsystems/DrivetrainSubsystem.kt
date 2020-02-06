package frc.robot.subsystems

import com.analog.adis16470.frc.ADIS16470_IMU
import com.revrobotics.CANEncoder
import com.revrobotics.CANPIDController
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel.MotorType
import com.revrobotics.ControlType
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import edu.wpi.first.wpilibj.util.Units
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.DrivetrainConstants
import frc.robot.util.applyDeadband
import frc.robot.util.constrain
import kotlin.math.abs

object DrivetrainSubsystem : SubsystemBase() {
  private val leftMasterMotor = CANSparkMax(DrivetrainConstants.LEFT_MASTER_ID, MotorType.kBrushless)
  private val leftSlaveMotor = CANSparkMax(DrivetrainConstants.LEFT_SLAVE_ID, MotorType.kBrushless)
  private val rightMasterMotor = CANSparkMax(DrivetrainConstants.RIGHT_MASTER_ID, MotorType.kBrushless)
  private val rightSlaveMotor = CANSparkMax(DrivetrainConstants.RIGHT_SLAVE_ID, MotorType.kBrushless)

  // Because getPIDController() uses constructor, which means heap allocation in a loop
  private val leftController = CANPIDController(leftMasterMotor)
  private val rightController = CANPIDController(rightMasterMotor)

  private val openLoopFeedforward = SimpleMotorFeedforward(0.0, 12.0 / DrivetrainConstants.MAX_SPEED)

  var closedLoopAccumError = 0.0;

  object Encoders {
    private val leftEncoders: Array<CANEncoder> = arrayOf(CANEncoder(leftMasterMotor), CANEncoder(leftSlaveMotor))
    private val rightEncoders: Array<CANEncoder> = arrayOf(CANEncoder(rightMasterMotor), CANEncoder(rightSlaveMotor))

    init {
      leftEncoders.forEach {
          it.velocityConversionFactor = DrivetrainConstants.DISTANCE_PER_REVOLUTION / 60.0
          it.positionConversionFactor = DrivetrainConstants.DISTANCE_PER_REVOLUTION
      }
      rightEncoders.forEach {
        it.velocityConversionFactor = DrivetrainConstants.DISTANCE_PER_REVOLUTION / 60.0
        it.positionConversionFactor = DrivetrainConstants.DISTANCE_PER_REVOLUTION
      }
    }

    /**
     * @return total left encoder position in meters
     */
    fun getLeftPosition(): Double {
      var sum = 0.0
      for (encoder in leftEncoders) {
        sum += encoder.position
      }
      return sum / leftEncoders.size
    }

    /**
     * @return total right encoder position in meters
     */
    fun getRightPosition(): Double {
      var sum = 0.0
      for (encoder in rightEncoders) {
        sum += encoder.position
      }
      return sum / leftEncoders.size
    }
  }

  private val odometry = DifferentialDriveOdometry(Rotation2d())
  val kinematics = DifferentialDriveKinematics(DrivetrainConstants.WHEEL_BASE_WIDTH)

  private val imu = ADIS16470_IMU()

  init {
    leftSlaveMotor.follow(leftMasterMotor)
    rightSlaveMotor.follow(rightMasterMotor)

    configureSMPID()
  }

  fun tankDrive(left: Double, right: Double, squareInputs: Boolean = true) {
    leftMasterMotor.setVoltage(
        if (squareInputs) {
          constrain(
              value = applyDeadband(value = left * abs(left), deadband = DrivetrainConstants.DEADBAND),
              max = 1.0) * 12.0
        } else {
          constrain(
              value = applyDeadband(value = left, deadband = DrivetrainConstants.DEADBAND),
              max = 1.0) * 12.0
        }
    )
    rightMasterMotor.setVoltage(
        if (squareInputs) {
          constrain(
              value = applyDeadband(value = right * abs(right), deadband = DrivetrainConstants.DEADBAND),
              max = 1.0) * 12.0
        } else {
          constrain(
              value = applyDeadband(value = right, deadband = DrivetrainConstants.DEADBAND),
              max = 1.0) * 12.0
        }
    )
  }

  fun tankDriveVolts(left: Double, right: Double, squareInputs: Boolean = false) {
    tankDrive(left = left / 12.0, right = right / 12.0, squareInputs = squareInputs)
  }

  /**
   * @param left target meters per second for left drivetrain
   * @param right target meters per second for right drivetrain
   */
  fun closedLoopDrive(left: Double, right: Double) {
    leftController.setReference(left, ControlType.kVelocity)
    rightController.setReference(right, ControlType.kVelocity)

    closedLoopAccumError = leftMasterMotor.pidController.iAccum.coerceAtLeast(rightMasterMotor.pidController.iAccum)
  }

  fun openLoopDrive(left: Double, right: Double) {
    leftMasterMotor.setVoltage(constrain(openLoopFeedforward.calculate(left), 12.0))
    rightMasterMotor.setVoltage(constrain(openLoopFeedforward.calculate(right), 12.0))
  }

  private fun configureSMPID() {
    for (controller in arrayOf(leftMasterMotor.pidController, rightMasterMotor.pidController)) {
      controller.p = DrivetrainConstants.SM_KP
      controller.i = DrivetrainConstants.SM_KI
      controller.d = DrivetrainConstants.SM_KD
      controller.ff = DrivetrainConstants.SM_KFF
      controller.iZone = DrivetrainConstants.SM_KI_ZONE
      controller.setSmartMotionMaxVelocity(DrivetrainConstants.SM_MAX_VEL, 0)
    }
  }

  @Override
  override fun periodic() {
    odometry.update(Rotation2d(getYaw()), Encoders.getLeftPosition(), Encoders.getRightPosition())
  }

  /**
   * @return angular velocity in radians per second
   */
  fun getAngularVelocity(): Double {
    return Units.degreesToRadians(imu.rate)
  }

  /**
   * @return angle in radians. Will go past 2pi
   */
  fun getYaw(): Double {
    return Units.degreesToRadians(imu.angle)
  }

  fun getPose(): Pose2d {
    return odometry.poseMeters
  }

  fun getRotation(): Rotation2d {
    return Rotation2d.fromDegrees(imu.angle)
  }

  fun resetOdometry(newPose: Pose2d, newAngle: Rotation2d) {
    odometry.resetPosition(newPose, newAngle)
  }
}