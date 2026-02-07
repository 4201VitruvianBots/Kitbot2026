// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.SetIntakeShooterSpeeds;
import frc.robot.constants.INTAKESHOOTER;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeShooter;
import frc.robot.subsystems.Vision;
import frc.team4201.lib.command.Auto;
import frc.robot.constants.CLIMBER.CLIMB_SPEED_PERCENT;
import frc.robot.subsystems.Climber;
import frc.robot.commands.SetClimbSpeed;

public class PreloadNeutralDepotClimb extends Auto {
  public PreloadNeutralDepotClimb(
          CommandSwerveDrivetrain swerveDrive, IntakeShooter intake, Vision vision, Climber climber) {
    try {
      var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

      var m_path1 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("PreloadNeutralDepotClimb1");
      var m_path2 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("PreloadNeutralDepotClimb2");
      var m_path3 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("PreloadNeutralDepotClimb3");
      var m_path4 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("PreloadNeutralDepotClimb4");
      var m_path5 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("PreloadNeutralDepotClimb5");
      var m_path6 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("PreloadNeutralDepotClimb6");
      var m_path7 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("PreloadNeutralDepotClimb7");

      addCommands(
          m_path1.andThen(() -> swerveDrive.setControl(stopRequest)),
          new SetIntakeShooterSpeeds(intake, INTAKESHOOTER.INTAKE_SPEED_PERCENT.SHOOT, INTAKESHOOTER.INTAKE_SPEED_PERCENT.KICKER_OUTAKE).withTimeout(3),
          m_path2.andThen(() -> swerveDrive.setControl(stopRequest)),
          new ParallelRaceGroup(
              new SetIntakeShooterSpeeds(intake, INTAKESHOOTER.INTAKE_SPEED_PERCENT.INTAKE, INTAKESHOOTER.INTAKE_SPEED_PERCENT.KICKER_INTAKE),
              m_path3.andThen(() -> swerveDrive.setControl(stopRequest))),
          m_path4.andThen(() -> swerveDrive.setControl(stopRequest)),
          new ParallelCommandGroup(
              new SetIntakeShooterSpeeds(intake, INTAKESHOOTER.INTAKE_SPEED_PERCENT.SHOOT, INTAKESHOOTER.INTAKE_SPEED_PERCENT.KICKER_OUTAKE).withTimeout(3),
              m_path5.andThen(() -> swerveDrive.setControl(stopRequest))),
          new ParallelRaceGroup(
              new SetIntakeShooterSpeeds(intake, INTAKESHOOTER.INTAKE_SPEED_PERCENT.INTAKE, INTAKESHOOTER.INTAKE_SPEED_PERCENT.KICKER_INTAKE),
              m_path6.andThen(() -> swerveDrive.setControl(stopRequest))),
          new SetClimbSpeed(climber, CLIMB_SPEED_PERCENT.UP).withTimeout(3),
          m_path7.andThen(() -> swerveDrive.setControl(stopRequest)),
          new SetClimbSpeed(climber, CLIMB_SPEED_PERCENT.DOWN).withTimeout(3)
          // Todo: add climb (command not yet implemented in this branch)
          );
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to load path for PreloadNeutralDepotClimb", e.getStackTrace());
      addCommands(new InstantCommand());
    }
  }
}
