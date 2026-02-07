// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.SetIntakeShooterSpeeds;
import frc.robot.constants.INTAKESHOOTER;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeShooter;
import frc.robot.subsystems.Vision;
import frc.team4201.lib.command.Auto;
import frc.team4201.lib.utils.TrajectoryUtils;
import java.util.function.BooleanSupplier;

public class PreloadNeutralShootTwice extends Auto {
  public PreloadNeutralShootTwice(
      CommandSwerveDrivetrain swerveDrive,
      IntakeShooter intake,
      Vision vision,
      BooleanSupplier flipPath) {
    try {
      var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

      TrajectoryUtils trajectoryUtils = swerveDrive.getTrajectoryUtils();

      var m_path1 = PathPlannerPath.fromPathFile("PreloadNeutralShootTwice1");
      var m_path2 = PathPlannerPath.fromPathFile("PreloadNeutralShootTwice2");
      var m_path3 = PathPlannerPath.fromPathFile("PreloadNeutralShootTwice3");
      var m_path4 = PathPlannerPath.fromPathFile("PreloadNeutralShootTwice4");

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
          new SetIntakeShooterSpeeds(intake, INTAKESHOOTER.INTAKE_SPEED_PERCENT.SHOOT, INTAKESHOOTER.INTAKE_SPEED_PERCENT.KICKER_OUTAKE).withTimeout(3),
          // This code just repeats the last four steps again.
          getPathCommand(trajectoryUtils, m_path2, flipPath)
              .andThen(() -> swerveDrive.setControl(stopRequest)),
          new ParallelRaceGroup(
              new SetIntakeShooterSpeeds(intake, INTAKESHOOTER.INTAKE_SPEED_PERCENT.INTAKE, INTAKESHOOTER.INTAKE_SPEED_PERCENT.KICKER_INTAKE),
              getPathCommand(trajectoryUtils, m_path3, flipPath)
                  .andThen(() -> swerveDrive.setControl(stopRequest))),
          getPathCommand(trajectoryUtils, m_path4, flipPath)
              .andThen(() -> swerveDrive.setControl(stopRequest)),
          new SetIntakeShooterSpeeds(intake, INTAKESHOOTER.INTAKE_SPEED_PERCENT.SHOOT, INTAKESHOOTER.INTAKE_SPEED_PERCENT.KICKER_OUTAKE).withTimeout(3));
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to load path for PreloadNeutralShootTwice", e.getStackTrace());
      addCommands(new InstantCommand());
    }
  }
}
