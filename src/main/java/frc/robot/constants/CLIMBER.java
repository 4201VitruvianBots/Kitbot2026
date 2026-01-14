package frc.robot.constants;

public class CLIMBER {
    //TODO: change values if needed
    public static final double kP = 1.00;
    public static final double kI = 1.00;
    public static final double kD = 1.00;
    public static final double gearRatio = 1.0 / 1.0;
    public static final double peakForwardOutput = 0.8;
    public static final double peakReverseOutput = -0.8;



    public enum CLIMB_SPEED_PERCENT {
        UP(0.3),
        DOWN(-0.2);

        private final double percent;

        CLIMB_SPEED_PERCENT(final double percent) {
        this.percent = percent;
        }

        public double get() {
        return percent;
        }
    }

}
