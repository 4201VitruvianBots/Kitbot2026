package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SWERVE;
import frc.robot.constants.FIELD;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.Vision;

public class ShootOnTheMove extends Command {

  private final Vision m_vision;
  private final CommandSwerveDrivetrain m_swerveDrivetrain;
  private final DoubleSupplier m_throttleInput;
  private final DoubleSupplier m_turnInput;

  private Double kTeleP_Theta = 1.0;
  private Double kTeleD_Theta = 0.0;
  public static final double kTeleI_Theta = 0.0;

  private PIDController m_PidController =
    new PIDController(kTeleP_Theta, kTeleI_Theta, kTeleD_Theta);


  Translation2d m_goal = new Translation2d();

  public ShootOnTheMove(Vision vision, CommandSwerveDrivetrain swerveDrive, DoubleSupplier throttleInput, DoubleSupplier turnInput) {
    m_vision = vision;
    m_swerveDrivetrain = swerveDrive;
    m_throttleInput = throttleInput;
    m_turnInput = turnInput;

    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Controls.isBlueAlliance()) {
      m_goal = FIELD.blueHub;
    } else {
      m_goal = FIELD.redHub;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d currentPose = m_swerveDrivetrain.getState().Pose.getTranslation();
    double robotToTargetDistance = m_goal.getDistance(currentPose);

    double PositionY = m_swerveDrivetrain.getState().Pose.getY();
    double PositionX = m_swerveDrivetrain.getState().Pose.getX();

    var chassisSpeeds = m_swerveDrivetrain.getState().Speeds;
    double VelocityY = chassisSpeeds.vyMetersPerSecond;
    double VelocityX = chassisSpeeds.vxMetersPerSecond;

    double AccelerationX = m_swerveDrivetrain.getPigeon2().getAccelerationX().getValueAsDouble();
    double AccelerationY = m_swerveDrivetrain.getPigeon2().getAccelerationY().getValueAsDouble();

    double VelocityShoot = (robotToTargetDistance * 3.281) / 1.2; 

    double virtualGoalX = m_goal.getX() - VelocityShoot * (VelocityX + AccelerationX);
    double virtualGoalY = m_goal.getY() - VelocityShoot * (VelocityY + AccelerationY);

    SmartDashboard.putNumber("Goal X", virtualGoalX);
    SmartDashboard.putNumber("Goal Y", virtualGoalY);
    SmartDashboard.putNumber("Distance to Hub", m_vision.getDistancetoHub().in(Meters));

    Translation2d movingGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);

    Translation2d toMovingGoal = movingGoalLocation.minus(currentPose);

    double newDist = toMovingGoal.getDistance(new Translation2d());

    double getOffsetAngleDeg =
        Math.asin((VelocityY * PositionX + VelocityX * PositionY) / (newDist * robotToTargetDistance));

    var targetDelta = m_goal.minus(m_swerveDrivetrain.getState().Pose.getTranslation()).getAngle();

    // all of the logic for angle is above this Comment

    var turnRate = m_PidController.calculate(m_swerveDrivetrain.getState().Pose.getRotation().getRadians(),
                    targetDelta.getRadians() + getOffsetAngleDeg);

    final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(SWERVE.kMaxSpeedMetersPerSecond * 0.1)
            .withRotationalDeadband(
                SWERVE.kMaxRotationRadiansPerSecond * 0.1) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    m_swerveDrivetrain.setControl(
        drive
            .withVelocityX((m_throttleInput.getAsDouble()) * SWERVE.kMaxSpeedMetersPerSecond)
            .withVelocityY((m_turnInput.getAsDouble()) * SWERVE.kMaxSpeedMetersPerSecond)
            .withRotationalRate(
                turnRate));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
