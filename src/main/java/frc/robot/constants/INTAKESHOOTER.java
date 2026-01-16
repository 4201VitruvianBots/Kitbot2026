package frc.robot.constants;

public class INTAKESHOOTER {
    public static final double peakForwardOutput = 0.9;
    public static final double peakReverseOutput = -0.9;

    public enum INTAKE_SPEED_PERCENT {
        INTAKE(-0.5),
        KICKER_INTAKE(-0.8),
        SHOOT(-0.7),
        KICKER_OUTAKE(0.9);

        private final double percent;

        INTAKE_SPEED_PERCENT(final double percent) {
        this.percent = percent;
        }

        public double get() {
        return percent;
        }
    }

}
