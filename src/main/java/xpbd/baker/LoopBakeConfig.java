package xpbd.baker;

/** 单次循环烘焙的不可变收敛配置；构造时会校验所有阈值。 */
public record LoopBakeConfig(
        int minimumWarmupCycles, int maximumWarmupCycles,
        int requiredStableCycles,
        double positionTolerance, double rotationToleranceRadians,
        double linearVelocityTolerance, double angularVelocityTolerance,
        double maximumPenetrationTolerance,
        boolean seamFallbackEnabled) {

    public LoopBakeConfig(int minimumWarmupCycles, int maximumWarmupCycles,
                          int requiredStableCycles,
                          double positionTolerance,
                          double rotationToleranceRadians,
                          double linearVelocityTolerance,
                          double angularVelocityTolerance,
                          boolean seamFallbackEnabled) {
        this(minimumWarmupCycles, maximumWarmupCycles, requiredStableCycles,
                positionTolerance, rotationToleranceRadians,
                linearVelocityTolerance, angularVelocityTolerance,
                Double.POSITIVE_INFINITY, seamFallbackEnabled);
    }

    public LoopBakeConfig {
        if (minimumWarmupCycles < 1
                || maximumWarmupCycles < minimumWarmupCycles
                || requiredStableCycles < 1) {
            throw new IllegalArgumentException("invalid loop cycle limits");
        }
        requireTolerance(positionTolerance, "position tolerance");
        requireTolerance(rotationToleranceRadians, "rotation tolerance");
        requireTolerance(linearVelocityTolerance, "linear velocity tolerance");
        requireTolerance(angularVelocityTolerance, "angular velocity tolerance");
        if (Double.isNaN(maximumPenetrationTolerance)
                || maximumPenetrationTolerance < 0) {
            throw new IllegalArgumentException(
                    "maximum penetration tolerance must be non-negative");
        }
    }

    public static LoopBakeConfig from(BoneMapper.PhysicsGroupConfig config) {
        return new LoopBakeConfig(
                config.minimumWarmupCycles, config.maximumWarmupCycles,
                config.requiredStableCycles, config.loopPositionTolerance,
                Math.toRadians(config.loopRotationToleranceDegrees),
                config.loopLinearVelocityTolerance,
                config.loopAngularVelocityTolerance,
                config.simulationMode == BoneMapper.SimulationMode.RIGID_BODY
                        ? config.rigidBodyMaximumSafePenetration
                        : config.collisionSkin,
                config.loopSeamFallbackEnabled);
    }

    private static void requireTolerance(double value, String label) {
        if (!Double.isFinite(value) || value < 0) {
            throw new IllegalArgumentException(label + " must be finite and non-negative");
        }
    }
}
