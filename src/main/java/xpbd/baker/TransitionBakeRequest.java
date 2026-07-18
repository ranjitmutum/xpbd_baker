package xpbd.baker;

import xpbd.loader.BedrockAnimationData;

import java.util.Map;
import java.util.Objects;

/** 同一会话内物理过渡片段的显式动画对请求。 */
public record TransitionBakeRequest(
        BedrockAnimationData.Animation sourceAnimation,
        BedrockAnimationData.Animation targetAnimation,
        double sourceExitTime, double targetEntryTime,
        double transitionDuration, Map<String, Double> perBoneFollowWeight) {

    public TransitionBakeRequest {
        Objects.requireNonNull(sourceAnimation, "sourceAnimation");
        Objects.requireNonNull(targetAnimation, "targetAnimation");
        if (!Double.isFinite(sourceAnimation.animationLength)
                || sourceAnimation.animationLength < 0
                || !Double.isFinite(targetAnimation.animationLength)
                || targetAnimation.animationLength < 0
                || !finiteInRange(sourceExitTime, 0, sourceAnimation.animationLength)
                || !finiteInRange(targetEntryTime, 0, targetAnimation.animationLength)
                || !Double.isFinite(transitionDuration)
                || !(transitionDuration > 0)) {
            throw new IllegalArgumentException("invalid transition timing");
        }
        perBoneFollowWeight = perBoneFollowWeight == null
                ? Map.of() : Map.copyOf(perBoneFollowWeight);
        for (Map.Entry<String, Double> entry : perBoneFollowWeight.entrySet()) {
            if (entry.getKey() == null || entry.getKey().isBlank()
                    || entry.getValue() == null
                    || !Double.isFinite(entry.getValue())
                    || entry.getValue() < 0 || entry.getValue() > 1) {
                throw new IllegalArgumentException(
                        "per-bone transition weights must be in [0, 1]");
            }
        }
    }

    public static TransitionBakeRequest endingAtClipBoundary(
            BedrockAnimationData.Animation source,
            BedrockAnimationData.Animation target, double duration) {
        return new TransitionBakeRequest(source, target,
                Math.max(0, source.animationLength), 0, duration, Map.of());
    }

    public double followWeight(String boneName) {
        return perBoneFollowWeight.getOrDefault(boneName, 1.0);
    }

    private static boolean finiteInRange(double value, double minimum,
                                         double maximum) {
        return Double.isFinite(value) && Double.isFinite(maximum)
                && value >= minimum && value <= Math.max(minimum, maximum) + 1e-12;
    }
}
