// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetIntakeShooterSpeeds;
import frc.robot.constants.INTAKESHOOTER.INTAKE_SPEED_PERCENT;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeShooter;

public class DepotLeft extends SequentialCommandGroup {
  public DepotLeft(CommandSwerveDrivetrain swerveDrive, IntakeShooter intakeShooter) {
    try {
      var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

      var m_path1 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("DepotLeftPath1");
      var m_path2 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("DepotLeftPath2");
      
      addCommands(
        new ParallelCommandGroup(
          m_path1.andThen(() -> swerveDrive.setControl(stopRequest)),
          new SetIntakeShooterSpeeds(intakeShooter, INTAKE_SPEED_PERCENT.INTAKE, INTAKE_SPEED_PERCENT.KICKER_INTAKE)
        ).withTimeout(3),
        m_path2.andThen(() -> swerveDrive.setControl(stopRequest)),
        new SetIntakeShooterSpeeds(intakeShooter, INTAKE_SPEED_PERCENT.SHOOT, INTAKE_SPEED_PERCENT.KICKER_OUTAKE)
      );
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path for DepotLeftPath", e.getStackTrace());
      addCommands(new InstantCommand());
    }
  }
}