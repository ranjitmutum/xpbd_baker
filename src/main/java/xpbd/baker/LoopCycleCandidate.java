package xpbd.baker;

/** 单个已完成预热循环的轻量级诊断数据。 */
public record LoopCycleCandidate(
        int cycleIndex,
        double normalizedScore,
        LoopErrorReport report) {

    public LoopCycleCandidate {
        if (cycleIndex < 1) {
            throw new IllegalArgumentException("cycle index must be positive");
        }
        if (Double.isNaN(normalizedScore) || normalizedScore < 0) {
            throw new IllegalArgumentException("cycle score must be non-negative");
        }
    }

    public boolean isBetterThan(LoopCycleCandidate other) {
        return other == null || normalizedScore < other.normalizedScore
                || (normalizedScore == other.normalizedScore
                && cycleIndex < other.cycleIndex);
    }
}
