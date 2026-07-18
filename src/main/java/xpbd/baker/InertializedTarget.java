package xpbd.baker;

/** 目标空间向量偏移的解析临界阻尼衰减。 */
public final class InertializedTarget {
    // 求解 (1 + x) * exp(-x) = 0.01，以得到零初速临界响应。
    private static final double ONE_PERCENT_CRITICAL_FACTOR = 6.638352067993813;
    private final double[] initialOffset;
    private final double[] initialVelocity;
    private final double duration;
    private final double omega;

    public InertializedTarget(double[] initialOffset, double[] initialVelocity,
                              double duration, double followWeight) {
        this.initialOffset = finiteVector(initialOffset, "initial offset");
        this.initialVelocity = finiteVector(initialVelocity, "initial velocity");
        if (this.initialOffset.length != this.initialVelocity.length
                || !Double.isFinite(duration) || !(duration > 0)
                || !Double.isFinite(followWeight)
                || followWeight < 0 || followWeight > 1) {
            throw new IllegalArgumentException("invalid inertialized target settings");
        }
        this.duration = duration;
        omega = followWeight == 0 ? 0
                : ONE_PERCENT_CRITICAL_FACTOR * followWeight / duration;
    }

    public double[] offsetAt(double elapsed) {
        double time = Double.isFinite(elapsed) ? Math.max(0, elapsed) : 0;
        double[] result = new double[initialOffset.length];
        if (omega == 0) {
            System.arraycopy(initialOffset, 0, result, 0, result.length);
            return result;
        }
        if (time >= duration) return result;

        double decay = Math.exp(-omega * time);
        double endDecay = Math.exp(-omega * duration);
        double u = time / duration;
        double u2 = u * u;
        double u3 = u2 * u;
    // 三次 Hermite 端点修正会保留 t=0 的原值和速度，同时使 t=T 的偏移和速度精确归零。
        double endValueWeight = -2 * u3 + 3 * u2;
        double endVelocityWeight = u3 - u2;
        for (int i = 0; i < result.length; i++) {
            double b = initialVelocity[i] + omega * initialOffset[i];
            double raw = (initialOffset[i] + b * time) * decay;
            double rawEnd = (initialOffset[i] + b * duration) * endDecay;
            double rawEndVelocity = (initialVelocity[i]
                    - omega * b * duration) * endDecay;
            result[i] = raw - endValueWeight * rawEnd
                    - endVelocityWeight * duration * rawEndVelocity;
        }
        return result;
    }

    private static double[] finiteVector(double[] value, String label) {
        if (value == null) throw new IllegalArgumentException(label + " is required");
        double[] result = value.clone();
        for (double component : result) {
            if (!Double.isFinite(component)) {
                throw new IllegalArgumentException(label + " must be finite");
            }
        }
        return result;
    }
}
