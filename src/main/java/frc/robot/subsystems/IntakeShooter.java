// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.INTAKESHOOTER;
import frc.robot.constants.INTAKESHOOTER.INTAKE_SPEED_PERCENT;

public class IntakeShooter extends SubsystemBase {
  private final TalonFX m_intakeMotor = new TalonFX(CAN.intakeMotor);
  private final TalonFX m_flywheelMotor = new TalonFX(CAN.flywheelMotor);

  /** Creates a new ExampleSubsystem. */
  public IntakeShooter() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = INTAKESHOOTER.kP;
    config.Slot0.kI = INTAKESHOOTER.kI;
    config.Slot0.kD = INTAKESHOOTER.kD;
    config.Feedback.SensorToMechanismRatio = INTAKESHOOTER.gearRatio;
    config.MotorOutput.PeakForwardDutyCycle = INTAKESHOOTER.peakForwardOutput;
    config.MotorOutput.PeakReverseDutyCycle = INTAKESHOOTER.peakReverseOutput;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  //TODO: UPDATE THE CODEX TO 2026
  //CtreUtils.configureTalonFx(m_intakeMotor, config);
  //CtreUtils.configureTalonFx(m_flywheelMotor, config);
   }

   // Motor speeds in percent
  public void setMotorSpeeds(double percent1, double percent2) {
    m_intakeMotor.set(percent1);
    m_flywheelMotor.set(percent2);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
