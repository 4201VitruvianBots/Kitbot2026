package frc.team4201.lib.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Generic interface that extends the WPILib Subsystem to incorporate functions for PathPlanner. */
public interface SwerveSubsystem extends Subsystem {

  /**
   * Function to get the module locations as a {@link Translation2d} array
   *
   * @return Translation2d[]
   */
  Translation2d[] getModuleLocations();

  /**
   * Function to get the PathPlanner {@link RobotConfig}
   *
   * @return RobotConfig
   */
  RobotConfig getAutoRobotConfig();

  /**
   * Function to get the translation {@link PIDConstants} for PathPlanner
   *
   * @return PIDConstants
   */
  PIDConstants getAutoTranslationPIDConstants();

  /**
   * Function to get the rotation {@link PIDConstants} for PathPlanner
   *
   * @return PIDConstants
   */
  PIDConstants getAutoRotationPIDConstants();

  /**
   * Function to get the CTRE SwerveDriveState. Used to get the Robot's {@link Pose2d}.
   *
   * @return SwerveDriveState
   */
  SwerveDriveState getState();

  /**
   * Function to reset the robot's pose given a {@link Pose2d}
   *
   * @param pose Robot's position on the field in {@link Pose2d}
   */
  void resetPose(Pose2d pose);

  /**
   * Function for PathPlanner to control the robot's motion in auto.
   *
   * @param chassisSpeeds WPILib's {@link ChassisSpeeds}
   * @param feedforwards PathPlanner's {@link DriveFeedforwards}
   */
  void setChassisSpeedsAuto(ChassisSpeeds chassisSpeeds, DriveFeedforwards feedforwards);
}