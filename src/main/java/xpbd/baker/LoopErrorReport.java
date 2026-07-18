package xpbd.baker;

/** 两个相邻周期边界状态的分单位诊断数据。 */
public record LoopErrorReport(
        double maximumPositionError, String positionBone,
        double maximumRotationErrorRadians, String rotationBone,
        double maximumLinearVelocityError, String linearVelocityBone,
        double maximumAngularVelocityError, String angularVelocityBone,
        boolean contactSetChanged, int contactDifferenceCount,
        double maximumPenetration, int anomalyCount) {

    public LoopErrorReport {
        requireNonNegativeOrUnavailable(maximumPositionError, "position error");
        requireNonNegativeOrUnavailable(maximumRotationErrorRadians, "rotation error");
        requireNonNegativeOrUnavailable(maximumLinearVelocityError,
                "linear velocity error");
        requireNonNegativeOrUnavailable(maximumAngularVelocityError,
                "angular velocity error");
        if (!Double.isFinite(maximumPenetration) || maximumPenetration < 0
                || contactDifferenceCount < 0 || anomalyCount < 0) {
            throw new IllegalArgumentException("invalid loop contact diagnostics");
        }
    }

    public boolean isWithin(LoopBakeConfig config) {
        return within(maximumPositionError, config.positionTolerance())
                && within(maximumRotationErrorRadians, config.rotationToleranceRadians())
                && within(maximumLinearVelocityError,
                config.linearVelocityTolerance())
                && within(maximumAngularVelocityError,
                config.angularVelocityTolerance())
                && !contactSetChanged
                && maximumPenetration <= config.maximumPenetrationTolerance()
                && anomalyCount == 0;
    }

    public String worstBone() {
        double maximum = -1;
        String bone = null;
        double[] values = {maximumPositionError, maximumRotationErrorRadians,
                maximumLinearVelocityError, maximumAngularVelocityError};
        String[] names = {positionBone, rotationBone,
                linearVelocityBone, angularVelocityBone};
        for (int i = 0; i < values.length; i++) {
            if (Double.isFinite(values[i]) && values[i] > maximum) {
                maximum = values[i];
                bone = names[i];
            }
        }
        return bone;
    }

    /** 各物理单位按自身容差归一化后的最大误差。 */
    public double normalizedScore(LoopBakeConfig config) {
        double maximum = Math.max(
                normalized(maximumPositionError, config.positionTolerance()),
                normalized(maximumRotationErrorRadians,
                        config.rotationToleranceRadians()));
        maximum = Math.max(maximum, normalized(maximumLinearVelocityError,
                config.linearVelocityTolerance()));
        maximum = Math.max(maximum, normalized(maximumAngularVelocityError,
                config.angularVelocityTolerance()));
        maximum = Math.max(maximum, normalized(maximumPenetration,
                config.maximumPenetrationTolerance()));
        double contactPenalty = contactSetChanged
                ? Math.max(1, contactDifferenceCount) : 0;
        return maximum + contactPenalty + anomalyCount;
    }

    /** 具有最大容差归一化姿态/速度误差的骨骼。 */
    public String worstBone(LoopBakeConfig config) {
        double maximum = -1;
        String bone = null;
        double[] values = {
                normalized(maximumPositionError, config.positionTolerance()),
                normalized(maximumRotationErrorRadians,
                        config.rotationToleranceRadians()),
                normalized(maximumLinearVelocityError,
                        config.linearVelocityTolerance()),
                normalized(maximumAngularVelocityError,
                        config.angularVelocityTolerance())
        };
        String[] names = {positionBone, rotationBone,
                linearVelocityBone, angularVelocityBone};
        for (int i = 0; i < values.length; i++) {
            if (Double.isFinite(values[i]) && values[i] > maximum) {
                maximum = values[i];
                bone = names[i];
            }
        }
        return bone;
    }

    private static boolean within(double value, double tolerance) {
        return Double.isNaN(value) || value <= tolerance;
    }

    private static double normalized(double value, double tolerance) {
        if (Double.isNaN(value)) return 0;
        if (tolerance == Double.POSITIVE_INFINITY) return 0;
        if (tolerance == 0) return value == 0 ? 0 : Double.POSITIVE_INFINITY;
        return value / tolerance;
    }

    private static void requireNonNegativeOrUnavailable(double value, String label) {
        if ((!Double.isFinite(value) && !Double.isNaN(value))
                || (Double.isFinite(value) && value < 0)) {
            throw new IllegalArgumentException(label + " must be non-negative or unavailable");
        }
    }
}
