package frc.robot.utils

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints
import frc.robot.constants.DrivetrainConstants
import frc.robot.subsystems.SwerveSubsystem

class PoseUtils {
    /**
     * Drives the robot to a position
     * @param target The target pose3d
     * @param current The current pose of what we are trying to move (probably the robot)
     * @param driveSubsystem The instance of the SwerveSubsystem for whatever command you are using. (Mabey doing this was a bad idea)
     */

    private val drivePID = ProfiledPIDController(DrivetrainConstants.alignP, DrivetrainConstants.alignI, DrivetrainConstants.alignD, Constraints(2.0, 1.0))
    private val rotationPID = ProfiledPIDController(DrivetrainConstants.alignP, DrivetrainConstants.alignI, DrivetrainConstants.alignD, Constraints(2.0, 1.0))

    fun GotoPose(target: Pose3d, current: Pose3d, driveSubsystem: SwerveSubsystem) {
        val distance = current.translation.getDistance(target.translation)

        var forwardOut = drivePID.calculate(distance, 0.0)
        if (drivePID.atGoal()) forwardOut = 0.0

        var rotationOut = rotationPID.calculate(current.rotation.angle, target.rotation.angle)
        if (drivePID.atGoal()) rotationOut = 0.0

        val drive = Pose2d(Translation2d(), current.translation.minus(target.translation).toTranslation2d().angle).transformBy(
            Transform2d(Translation2d(forwardOut, 0.0), Rotation2d())
        )

        driveSubsystem.drive(drive.x, drive.y, rotationOut, false, true)

    }
}

