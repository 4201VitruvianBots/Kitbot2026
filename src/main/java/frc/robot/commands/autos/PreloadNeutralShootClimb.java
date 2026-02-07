// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.SetClimbSpeed;
import frc.robot.commands.SetIntakeShooterSpeeds;
import frc.robot.constants.CLIMBER.CLIMB_SPEED_PERCENT;
import frc.robot.constants.INTAKESHOOTER;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeShooter;
import frc.robot.subsystems.Vision;
import frc.team4201.lib.command.Auto;
import frc.team4201.lib.utils.TrajectoryUtils;
import frc.robot.subsystems.Climber;

import java.util.function.BooleanSupplier;

public class PreloadNeutralShootClimb extends Auto {
  public PreloadNeutralShootClimb(
      CommandSwerveDrivetrain swerveDrive,
      IntakeShooter intake,
      Vision vision,
      Climber climber,
      BooleanSupplier flipPath) {
    try {
      var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

      TrajectoryUtils trajectoryUtils = swerveDrive.getTrajectoryUtils();

      var m_path1 = PathPlannerPath.fromPathFile("PreloadNeutralShootClimb1");
      var m_path2 = PathPlannerPath.fromPathFile("PreloadNeutralShootClimb2");
      var m_path3 = PathPlannerPath.fromPathFile("PreloadNeutralShootClimb3");
      var m_path4 = PathPlannerPath.fromPathFile("PreloadNeutralShootClimb4");

      addCommands(
          getPathCommand(trajectoryUtils, m_path1, flipPath)
              .andThen(() -> swerveDrive.setControl(stopRequest)),
          new SetIntakeShooterSpeeds(intake, INTAKESHOOTER.INTAKE_SPEED_PERCENT.SHOOT, INTAKESHOOTER.INTAKE_SPEED_PERCENT.KICKER_OUTAKE).withTimeout(3),
          getPathCommand(trajectoryUtils, m_path2, flipPath)
              .andThen(() -> swerveDrive.setControl(stopRequest)),
          new ParallelRaceGroup(
              new SetIntakeShooterSpeeds(intake, INTAKESHOOTER.INTAKE_SPEED_PERCENT.INTAKE, INTAKESHOOTER.INTAKE_SPEED_PERCENT.KICKER_INTAKE),
              getPathCommand(trajectoryUtils, m_path3, flipPath)
                  .andThen(() -> swerveDrive.setControl(stopRequest))),
          getPathCommand(trajectoryUtils, m_path4, flipPath)
              .andThen(() -> swerveDrive.setControl(stopRequest)),
          new SetIntakeShooterSpeeds(intake, INTAKESHOOTER.INTAKE_SPEED_PERCENT.SHOOT, INTAKESHOOTER.INTAKE_SPEED_PERCENT.KICKER_OUTAKE).withTimeout(3)
      );
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to load path for PreloadNeutralShootClimb", e.getStackTrace());
      addCommands(new InstantCommand());
    }
  }
}
