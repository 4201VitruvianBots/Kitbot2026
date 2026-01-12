package frc.robot.constants;

public class INTAKESHOOTER {
    //TODO: change values if needed
    public static final double kP = 1.00;
    public static final double kI = 1.00;
    public static final double kD = 1.00;
    public static final double gearRatio = 1.0 / 1.0;
    public static final double peakForwardOutput = 0.8;
    public static final double peakReverseOutput = -0.8;

    public enum INTAKE_SPEED_PERCENT {
        INTAKE(-0.5),
        SHOOT(0.6);

        private final double percent;

        INTAKE_SPEED_PERCENT(final double percent) {
        this.percent = percent;
        }

        public double get() {
        return percent;
        }
    }

}
