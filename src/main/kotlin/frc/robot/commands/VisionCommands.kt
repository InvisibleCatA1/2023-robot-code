package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.wpilibj.GenericHID.RumbleType
import edu.wpi.first.wpilibj.XboxController
import frc.robot.VisionUtils
import frc.robot.constants.VisionConstants
import frc.robot.subsystems.SwerveSubsystem
import edu.wpi.first.wpilibj2.command.*
import frc.robot.constants.DrivetrainConstants
import frc.robot.subsystems.GyroSubsystem
import frc.robot.utils.PoseUtils

fun SetPipeline(pipeline: VisionConstants.Pipelines) : Command {
    return InstantCommand({
        VisionUtils.setPipelineIndex("", pipeline.ordinal)
    })
}
class RumbleCheck(val controller: XboxController, val check: () -> Boolean): CommandBase() {
    override fun execute(){
        controller.setRumble(RumbleType.kBothRumble, (if (check()) 1.0 else 0.0));
    }

    override fun end(interrupted: Boolean) {
        controller.setRumble(RumbleType.kBothRumble, 0.0)
    }

    override fun isFinished(): Boolean {
        return !check()
  }
}

/**
 * Align to a vision target
 * @param driveSystem SwerveSubsystem instance
 * @param gyro GyroSubsystem instance
 * @param april If we are trying to align to an april tag or not (default: false)
 * @param targetArea The target area (ta) value when the object is the target distance away (default: 0.0).
 * 
 * */
class AlignToTarget(val driveSystem: SwerveSubsystem, val gyro: GyroSubsystem, val april: Boolean = true, val targetArea: Double = 0.0): CommandBase() {
    private var robotposeT: Pose3d? = null
    private var robotPose = Pose3d(driveSystem.pose)
    private val poseUtils = PoseUtils()

    // PIDs
    private val rotationPID = PIDController(DrivetrainConstants.rotationP, DrivetrainConstants.rotationI, DrivetrainConstants.rotationD)
    private val forwardPID = PIDController(DrivetrainConstants.fowardP, DrivetrainConstants.fowardI, DrivetrainConstants.fowardD)

    init {
        addRequirements(driveSystem)
        if (!april) {
            addRequirements(gyro)
            gyro.reset()
        }
        rotationPID.setpoint = 0.0
        forwardPID.setpoint = targetArea
    }

    override fun execute() {
        robotPose =  Pose3d(driveSystem.pose)

        if (april) {
            robotposeT =
                VisionUtils.getLatestResults("").targetingResults.targets_Fiducials[0]?.getRobotPose_TargetSpace()!!
            val targetPose = Pose3d(5.0, 0.0, robotposeT!!.z, Rotation3d(0.0 ,0.0 ,0.0))
            poseUtils.GotoPose(targetPose, robotposeT!!, driveSystem)
        } else {
            // TODO: Make normal align.
//            driveSystem.drive(0.0, 0.0, 0.0, false, false)
            val targetRotation = gyro.getHeading() + VisionUtils.getTX("")
            var rOut = rotationPID.calculate(targetRotation)
            var fOut = 0.0
            if (rotationPID.atSetpoint()) {
                rOut = 0.0
                fOut = forwardPID.calculate(VisionUtils.getTA(""))
            }

            driveSystem.drive(fOut, 0.0, rOut, false, true)




        }
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            println("[WARN] Align to target was interrupted")
        }
    }

    override fun isFinished() = false
}


fun AlignToAprilTag(driveSubsystem: SwerveSubsystem, gyro: GyroSubsystem,controller: XboxController): Command{
  return(SequentialCommandGroup(
      SetPipeline(VisionConstants.Pipelines.APRILTAG),
      RumbleCheck(controller) { !VisionUtils.getTV("") },
      AlignToTarget(driveSubsystem, gyro)
  ))
}

fun AlignToCone(driveSubsystem: SwerveSubsystem, gyro: GyroSubsystem, controller: XboxController): SequentialCommandGroup{
  return(SequentialCommandGroup(
    SetPipeline(VisionConstants.Pipelines.CONE),
    RumbleCheck(controller) { VisionUtils.getTV("") },
    AlignToTarget(driveSubsystem, gyro, false, 10.0)
  ))
}
fun AlignToRetroreflective(driveSubsystem: SwerveSubsystem, gyro: GyroSubsystem,controller: XboxController): SequentialCommandGroup{
  return(SequentialCommandGroup(
      SetPipeline(VisionConstants.Pipelines.RETROREFLECTIVE),
      RumbleCheck(controller) { VisionUtils.getTV("") },
      AlignToTarget(driveSubsystem, gyro, false, 10.0)
  ))
}

fun AlignToCube(driveSubsystem: SwerveSubsystem, gyro: GyroSubsystem,controller: XboxController): SequentialCommandGroup {
    return(SequentialCommandGroup(
        SetPipeline(VisionConstants.Pipelines.CUBE),
        RumbleCheck(controller) { VisionUtils.getTV("") },
        AlignToTarget(driveSubsystem, gyro, false, 10.0)
    ))
}