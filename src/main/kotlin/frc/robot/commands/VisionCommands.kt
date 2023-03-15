package frc.robot.commands

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.wpilibj.GenericHID.RumbleType
import edu.wpi.first.wpilibj.XboxController
import frc.robot.VisionUtils
import frc.robot.constants.VisionConstants
import frc.robot.subsystems.SwerveSubsystem
import edu.wpi.first.wpilibj2.command.*
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

class AlignToTarget(val driveSystem: SwerveSubsystem, val april: Boolean = true): CommandBase() {
    private var robotPose_t: Pose3d? = null
    private var robotPose = Pose3d(driveSystem.pose)
    private val poseUtils = PoseUtils()

    init {
        addRequirements(driveSystem)
    }

    override fun execute() {
        robotPose =  Pose3d(driveSystem.pose)

        if (april) {
            robotPose_t =
                VisionUtils.getLatestResults("").targetingResults.targets_Fiducials[0]?.getRobotPose_TargetSpace()!!

            val targetPose = Pose3d(5.0, 0.0, robotPose_t!!.z, Rotation3d(0.0 ,0.0 ,0.0))
            poseUtils.GotoPose(targetPose, robotPose_t!!, driveSystem)

        } else {
            // TODO: Make normal align.
            driveSystem.drive(0.0, 0.0, 0.0, false, false)
        }
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            println("[WARN] Align to target was interrupted")
        }
    }

    override fun isFinished() = false
}


fun AlignToAprilTag(driveSubsystem: SwerveSubsystem, controller: XboxController): Command{
  return(SequentialCommandGroup(
      SetPipeline(VisionConstants.Pipelines.APRILTAG),
      RumbleCheck(controller) { !VisionUtils.getTV("") },
      AlignToTarget(driveSubsystem)
  ))
}
//
//fun AlignToCone(driveSubsystem: SwerveSubsystem, controller: XboxController): SequentialCommandGroup{
//  return(SequentialCommandGroup(
//    SetPipeline(VisionConstants.Pipelines.CONE),
//    RumbleCheck(controller) { VisionUtils.getTV("") },
//    XAlign(driveSubsystem)
//  ))
//}
//fun AlignToRetroreflective(driveSubsystem: SwerveSubsystem, controller: XboxController): SequentialCommandGroup{
//  return(SequentialCommandGroup(
//    SetPipeline(VisionConstants.Pipelines.RETROREFLECTIVE),
//    RumbleCheck(controller) { VisionUtils.getTV("") },
//    XAlign(driveSubsystem)
//  ))
//}
//
//fun AlignToCube(driveSubsystem: SwerveSubsystem, controller: XboxController): SequentialCommandGroup {
//    return(SequentialCommandGroup(
//        SetPipeline(VisionConstants.Pipelines.CUBE),
//        RumbleCheck(controller) { VisionUtils.getTV("") },
//        XAlign(driveSubsystem)
//    ))
//}