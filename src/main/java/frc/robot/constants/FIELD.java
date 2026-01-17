package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;

public class FIELD {
  /**
   * FIELD
   *
   * <p>Field constants
   *
   * <p>Note: Values are using ideal values from WPILib TODO: Create layout from practice field.
   */
  public static final AprilTagFieldLayout wpilibAprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  //TODO: change to k2026RebuiltWelded when that gets released if ever
  // public static final AprilTagFieldLayout practiceFieldAprilTagLayout;

  public static final AprilTagFieldLayout aprilTagFieldLayout = wpilibAprilTagLayout;

  public static final Translation2d redHub = new Translation2d(4.625594,4.034536);
  public static final Translation2d blueHub = new Translation2d(11.915394, 4.034536);

  public static final Translation2d redAutoHub = redHub;
  public static final Translation2d blueAutoHub = blueHub;
  
  /** Field X-axis */
  public static final Distance LENGTH = Meters.of(aprilTagFieldLayout.getFieldLength());

  /** Field Y-axis */
  public static final Distance WIDTH = Meters.of(aprilTagFieldLayout.getFieldWidth());

  public static final Distance APRILTAG_SIZE = Inches.of(6.5);

  public static final Distance CORAL_STATION_HEIGHT = Inches.of(55.25).plus(APRILTAG_SIZE).div(2.0);
  public static final Distance PROCESSOR_HEIGHT = Inches.of(47.875).plus(APRILTAG_SIZE).div(2.0);
  public static final Distance BARGE_HEIGHT = Inches.of(70.75).plus(APRILTAG_SIZE).div(2.0);
  public static final Distance REEF_HEIGHT = Inches.of(8.75).plus(APRILTAG_SIZE).div(2.0);

  /** Enum describing all AprilTags on the field by ID and their Pose3d positions. */
  public enum APRIL_TAG {
    RED_TRENCH_NEAR_RIGHT(1),
    RED_HUB_NEAR_RIGHT(2),
    RED_HUB_RIGHT_CLOSE(3),
    RED_HUB_RIGHT_FAR(4),
    RED_HUB_FAR_RIGHT(5),
    RED_TRENCH_FAR_RIGHT(6),
    RED_TRENCH_FAR_LEFT(7),
    RED_HUB_FAR_LEFT(8),
    RED_HUB_LEFT_AWAY(9),
    RED_HUB_LEFT_CLOSE(10),
    RED_HUB_NEAR_LEFT(11),
    RED_TRENCH_NEAR_LEFT(12),
    RED_OUTPOST_NEAR(13),
    RED_OUTPOST_FAR(14),
    RED_TOWER_NEAR(15),
    RED_TOWER_FAR(16),
    BLUE_TRENCH_FAR_LEFT(17),
    BLUE_HUB_FAR_LEFT(18),
    BLUE_HUB_LEFT_FAR(19),
    BLUE_HUB_LEFT_NEAR(20),
    BLUE_HUB_NEAR_LEFT(21),
    BLUE_TRENCH_NEAR_LEFT(22),
    BLUE_TRENCH_NEAR_RIGHT(23),
    BLUE_HUB_NEAR_RIGHT(24),
    BLUE_HUB_RIGHT_NEAR(25),
    BLUE_HUB_RIGHT_FAR(26),
    BLUE_HUB_FAR_RIGHT(27),
    BLUE_TRENCH_FAR_RIGHT(28),
    BLUE_OUTPOST_FAR(29),
    BLUE_OUTPOST_NEAR(30),
    BLUE_TOWER_FAR(31),
    BLUE_TOWER_NEAR(32);

    private final int id;
    private Pose3d pose;

    APRIL_TAG(final int id) {
      this.id = id;
      aprilTagFieldLayout
          .getTagPose(this.id)
          .ifPresentOrElse(
              pose3d -> this.pose = pose3d,
              () -> {
                System.out.printf(
                    "[FIELD] Could not read AprilTag ID %s data from FieldLayout\n", this.id);
                new Alert(
                        String.format("[FIELD] APRIL_TAG ID %s value couldn't be read", this.id),
                        AlertType.kError)
                    .set(true);
              });

      if (this.pose == null) {
        throw new IllegalArgumentException("AprilTag ID " + this.id + " does not have a Pose3d!");
      }
    }

    public int getId() {
      return id;
    }

    public Pose3d getPose3d() {
      return pose;
    }

    public Pose2d getPose2d() {
      return pose.toPose2d();
    }

    public Translation3d getTranslation3d() {
      return pose.getTranslation();
    }

    public Translation2d getTranslation2d() {
      return getPose2d().getTranslation();
    }

    public static APRIL_TAG getTagById(int aprilTagId) {
      for (var t : APRIL_TAG.values()) {
        if (t.id == aprilTagId) {
          return t;
        }
      }
      throw new IllegalArgumentException("AprilTag ID " + aprilTagId + " does not exist!");
    }

    public static Pose2d[] getAllAprilTagPoses() {
      return Arrays.stream(APRIL_TAG.values()).map(APRIL_TAG::getPose2d).toArray(Pose2d[]::new);
    }
  }

  //
  // Coral Branch constants
  //

  /** Depth of the reef poles from the AprilTag */
  //  static Distance baseReefDepth = Inches.of(30.738);  // Mechanical Advantage's measurement
  //  static Distance baseReefDepth = Inches.of(24); // CAD Measurement to base
  static final Distance baseReefDepth = Inches.of(2); // CAD Measurement L4 CC distance

  // TODO: Verify these offset is correct
  /**
   * Left/right translation of the reef poles perpendicular to the center of the AprilTag's Pose2d.
   */
  static final Distance baseReefTranslation = Inches.of(7);

  static final Translation2d leftReefCoralOffset =
      new Translation2d(-baseReefDepth.in(Meters), baseReefTranslation.in(Meters));
  static final Translation2d rightReefCoralOffset =
      new Translation2d(-baseReefDepth.in(Meters), -baseReefTranslation.in(Meters));

  static final Translation2d baseLeftCoralTargetOffset =
      new Translation2d(
          SWERVE.kWheelBase.div(2).plus(SWERVE.kBumperThickness).minus(Inches.of(1.5)).in(Meters),
          baseReefTranslation.in(Meters));
  static final Translation2d baseRightCoralTargetOffset =
      new Translation2d(
          SWERVE.kWheelBase.div(2).plus(SWERVE.kBumperThickness).minus(Inches.of(1.5)).in(Meters),
          -baseReefTranslation.in(Meters));

  /**
   * Location of all coral targets based on their position relative to their nearest AprilTag.
   * Includes the ability to add a left/right offset in inches to account for field differences
   */
  public enum CORAL_TARGETS {
    RED_REEF_NEAR_LEFT_LEFT(6, true, Inches.of(0)),
    RED_REEF_NEAR_LEFT_RIGHT(6, false, Inches.of(0)),
    RED_REEF_NEAR_CENTER_LEFT(7, true, Inches.of(0)),
    RED_REEF_NEAR_CENTER_RIGHT(7, false, Inches.of(0)),
    RED_REEF_NEAR_RIGHT_LEFT(8, true, Inches.of(0)),
    RED_REEF_NEAR_RIGHT_RIGHT(8, false, Inches.of(0)),
    RED_REEF_FAR_RIGHT_LEFT(9, true, Inches.of(0)),
    RED_REEF_FAR_RIGHT_RIGHT(9, false, Inches.of(0)),
    RED_REEF_FAR_CENTER_LEFT(10, true, Inches.of(0)),
    RED_REEF_FAR_CENTER_RIGHT(10, false, Inches.of(0)),
    RED_REEF_FAR_LEFT_LEFT(11, true, Inches.of(0)),
    RED_REEF_FAR_LEFT_RIGHT(11, false, Inches.of(0)),
    BLUE_REEF_NEAR_RIGHT_LEFT(17, true, Inches.of(0)),
    BLUE_REEF_NEAR_RIGHT_RIGHT(17, false, Inches.of(0)),
    BLUE_REEF_NEAR_CENTER_LEFT(18, true, Inches.of(0)),
    BLUE_REEF_NEAR_CENTER_RIGHT(18, false, Inches.of(0)),
    BLUE_REEF_NEAR_LEFT_LEFT(19, true, Inches.of(0)),
    BLUE_REEF_NEAR_LEFT_RIGHT(19, false, Inches.of(0)),
    BLUE_REEF_FAR_LEFT_LEFT(20, true, Inches.of(0)),
    BLUE_REEF_FAR_LEFT_RIGHT(20, false, Inches.of(0)),
    BLUE_REEF_FAR_CENTER_LEFT(21, true, Inches.of(0)),
    BLUE_REEF_FAR_CENTER_RIGHT(21, false, Inches.of(0)),
    BLUE_REEF_FAR_RIGHT_LEFT(22, true, Inches.of(0)),
    BLUE_REEF_FAR_RIGHT_RIGHT(22, false, Inches.of(0));

    /** Pose2d of the coral */
    private final Pose2d pose;

    /** Pose2d we want the robot to go to relative to the branch's location */
    private final Pose2d targetPose;

    /** If the branch is a left branch (When facing the Reef) */
    private final boolean isLeftBranch;

    /** Map to convert the coral's Pose2d to the target Pose2d */
    private static final Map<Pose2d, Pose2d> coralPoseToTargetPose = new HashMap<>();

    CORAL_TARGETS(final int aprilTagId, boolean isLeft, Distance offset) {
      Pose2d branchPose = Pose2d.kZero;
      Pose2d targetPose = Pose2d.kZero;

      var branchOffset = isLeft ? leftReefCoralOffset : rightReefCoralOffset;
      branchOffset.plus(new Translation2d(0, offset.in(Meters)));
      var targetOffset = isLeft ? baseLeftCoralTargetOffset : baseRightCoralTargetOffset;
      try {
        var aprilTagPose = APRIL_TAG.getTagById(aprilTagId).getPose2d();
        branchPose = aprilTagPose.plus(new Transform2d(branchOffset, Rotation2d.kZero));

        targetPose = aprilTagPose.plus(new Transform2d(targetOffset, Rotation2d.kZero));
        targetPose =
            new Pose2d(
                targetPose.getTranslation(), targetPose.getRotation().plus(Rotation2d.k180deg));
      } catch (Exception e) {
        System.out.printf(
            "[FIELD] Could not get AprilTag %d Pose for CORAL_TARGET generation!\n", aprilTagId);
      }
      this.pose = branchPose;
      this.targetPose = targetPose;
      this.isLeftBranch = isLeft;
    }

    /** Map the coral pose to the target pose */
    static {
      for (CORAL_TARGETS b : CORAL_TARGETS.values()) {
        coralPoseToTargetPose.put(b.getPose2d(), b.getTargetPose2d());
      }
    }

    /** Return the selected coral's position as a Pose2d */
    public Pose2d getPose2d() {
      return pose;
    }

    /** Return the selected coral position's target as a Pose2d */
    public Pose2d getTargetPose2d() {
      return targetPose;
    }

    public boolean isLeftBranch() {
      return isLeftBranch;
    }

    /** 2d array of all coral positions */
    public static Pose2d[] getAllPose2d() {
      return Arrays.stream(CORAL_TARGETS.values())
          .map(CORAL_TARGETS::getPose2d)
          .toArray(Pose2d[]::new);
    }

    public static Pose2d[] getAllLeftPose2d() {
      return Arrays.stream(CORAL_TARGETS.values())
          .filter(CORAL_TARGETS::isLeftBranch)
          .map(CORAL_TARGETS::getPose2d)
          .toArray(Pose2d[]::new);
    }

    public static Pose2d[] getAllRightPose2d() {
      return Arrays.stream(CORAL_TARGETS.values())
          .filter(c -> !c.isLeftBranch())
          .map(CORAL_TARGETS::getPose2d)
          .toArray(Pose2d[]::new);
    }

    /** 2d array of coral positions by Alliance color */
    public static Pose2d[] getAlliancePose2d(DriverStation.Alliance alliance) {
      if (alliance == DriverStation.Alliance.Red) {
        return Arrays.stream(getAllPose2d()).limit(12).toArray(Pose2d[]::new);
      } else {
        return Arrays.stream(getAllPose2d()).skip(12).limit(12).toArray(Pose2d[]::new);
      }
    }

    /** 2d array of left coral positions by Alliance color */
    public static Pose2d[] getAllianceLeftPose2d(DriverStation.Alliance alliance) {
      if (alliance == DriverStation.Alliance.Red) {
        return Arrays.stream(getAllLeftPose2d()).limit(6).toArray(Pose2d[]::new);
      } else {
        return Arrays.stream(getAllLeftPose2d()).skip(6).limit(6).toArray(Pose2d[]::new);
      }
    }

    /** 2d array of right coral positions by Alliance color */
    public static Pose2d[] getAllianceRightPose2d(DriverStation.Alliance alliance) {
      if (alliance == DriverStation.Alliance.Red) {
        return Arrays.stream(getAllRightPose2d()).limit(6).toArray(Pose2d[]::new);
      } else {
        return Arrays.stream(getAllRightPose2d()).skip(6).limit(6).toArray(Pose2d[]::new);
      }
    }

    /** 2d array of all coral targets */
    public static Pose2d[] getAllTargetPose2d() {
      return Arrays.stream(CORAL_TARGETS.values())
          .map(CORAL_TARGETS::getTargetPose2d)
          .toArray(Pose2d[]::new);
    }

    /** 2d array of coral targets by Alliance color */
    public static Pose2d[] getAllianceTargetPose2d(DriverStation.Alliance alliance) {
      if (alliance == DriverStation.Alliance.Red) {
        return Arrays.stream(getAllTargetPose2d()).limit(12).toArray(Pose2d[]::new);
      } else {
        return Arrays.stream(getAllTargetPose2d()).skip(12).limit(12).toArray(Pose2d[]::new);
      }
    }

    /** Convert the nearest coral position to the target position */
    public static Pose2d getCoralPoseToTargetPose(Pose2d branchPose) {
      if (!coralPoseToTargetPose.containsKey(branchPose)) {
        DriverStation.reportWarning("[FIELD] Trying to use a non-existent coral pose!", false);
        return Pose2d.kZero;
      } else {
        return coralPoseToTargetPose.get(branchPose);
      }
    }
  }

  /** Static array of all Red Alliance Reef Branches to avoid constantly generating it */
  public static final List<Pose2d> RED_CORAL_BRANCHES =
      Arrays.asList(CORAL_TARGETS.getAlliancePose2d(DriverStation.Alliance.Red));

  /** Static array of all Blue Alliance Reef Branches to avoid constantly generating it */
  public static final List<Pose2d> BLUE_CORAL_BRANCHES =
      Arrays.asList(CORAL_TARGETS.getAlliancePose2d(DriverStation.Alliance.Blue));

  /** Static array of all Red Alliance Reef Branch targets to avoid constantly generating it */
  public static final List<Pose2d> RED_CORAL_TARGETS =
      Arrays.asList(CORAL_TARGETS.getAllianceTargetPose2d(DriverStation.Alliance.Red));

  /** Static array of all Blue Alliance Reef Branch targets to avoid constantly generating it */
  public static final List<Pose2d> BLUE_CORAL_TARGETS =
      Arrays.asList(CORAL_TARGETS.getAllianceTargetPose2d(DriverStation.Alliance.Blue));

  /** Static array of all Red Alliance Left Reef Branches to avoid constantly generating it */
  public static final List<Pose2d> RED_CORAL_LEFT_BRANCHES =
      Arrays.asList(CORAL_TARGETS.getAllianceLeftPose2d(DriverStation.Alliance.Red));

  /** Static array of all Red Alliance Right Reef Branches to avoid constantly generating it */
  public static final List<Pose2d> RED_CORAL_RIGHT_BRANCHES =
      Arrays.asList(CORAL_TARGETS.getAllianceRightPose2d(DriverStation.Alliance.Red));

  /** Static array of all Blue Alliance Left Reef Branches to avoid constantly generating it */
  public static final List<Pose2d> BLUE_CORAL_LEFT_BRANCHES =
      Arrays.asList(CORAL_TARGETS.getAllianceLeftPose2d(DriverStation.Alliance.Blue));

  /** Static array of all Blue Alliance Right Reef Branches to avoid constantly generating it */
  public static final List<Pose2d> BLUE_CORAL_RIGHT_BRANCHES =
      Arrays.asList(CORAL_TARGETS.getAllianceRightPose2d(DriverStation.Alliance.Blue));
}
