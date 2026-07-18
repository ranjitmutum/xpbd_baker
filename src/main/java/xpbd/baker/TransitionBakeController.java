package xpbd.baker;

import xpbd.loader.BedrockAnimationData;
import xpbd.loader.BedrockModelData;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** 在局部骨骼空间中，以从 A 到 B 的临界阻尼目标偏移采样 B。 */
public final class TransitionBakeController {
    private final TransitionBakeRequest request;
    private final List<BedrockModelData.Bone> bones;
    private final Map<String, BedrockModelData.Bone> bonesByName;
    private final Map<String, PhysicsBaker.BoneState> physicalStates;
    private final Map<String, PhysicsBaker.BoneState> previousPhysicalStates;
    private final double physicalFrameSpan;
    private final Map<String, BoneOffset> offsets = new LinkedHashMap<>();

    public TransitionBakeController(TransitionBakeRequest request,
                                    List<BedrockModelData.Bone> bones) {
        this(request, bones, null);
    }

    public TransitionBakeController(TransitionBakeRequest request,
                                    List<BedrockModelData.Bone> bones,
                                    PhysicsBaker.BakedFrame physicalFrame) {
        this(request, bones, physicalFrame, null, 0);
    }

    public TransitionBakeController(TransitionBakeRequest request,
                                    List<BedrockModelData.Bone> bones,
                                    PhysicsBaker.BakedFrame physicalFrame,
                                    PhysicsBaker.BakedFrame previousPhysicalFrame,
                                    double physicalFrameSpan) {
        this.request = request;
        this.bones = List.copyOf(bones);
        bonesByName = new HashMap<>();
        for (BedrockModelData.Bone bone : this.bones) {
            if (bone != null && bone.name != null) bonesByName.put(bone.name, bone);
        }
        physicalStates = new HashMap<>();
        previousPhysicalStates = new HashMap<>();
        indexStates(physicalFrame, physicalStates);
        indexStates(previousPhysicalFrame, previousPhysicalStates);
        this.physicalFrameSpan = Double.isFinite(physicalFrameSpan)
                && physicalFrameSpan > 0 ? physicalFrameSpan : 0;
        initializeOffsets();
    }

    public Map<String, BonePoseCalculator.Pose> sample(double elapsed) {
        double safeElapsed = Math.max(0, Math.min(
                request.transitionDuration(), elapsed));
        double targetTime = targetSampleTime(safeElapsed);
        Map<String, double[]> positionOverrides = new HashMap<>();
        Map<String, double[]> rotationOverrides = new HashMap<>();
        for (Map.Entry<String, BoneOffset> entry : offsets.entrySet()) {
            String name = entry.getKey();
            BoneOffset offset = entry.getValue();
            double[] targetPosition = channel(request.targetAnimation(),
                    name, targetTime, false);
            double[] positionOffset = offset.position().offsetAt(safeElapsed);
            for (int axis = 0; axis < 3; axis++) {
                targetPosition[axis] += positionOffset[axis];
            }
            positionOverrides.put(name, targetPosition);

            double[] targetEuler = channel(request.targetAnimation(),
                    name, targetTime, true);
            double[] targetTotalEuler = totalEuler(name, targetEuler);
            double[] targetQ = RotationUtil.quaternionFromBedrockEuler(
                    targetTotalEuler[0], targetTotalEuler[1], targetTotalEuler[2]);
            double[] rotationVector = offset.rotation().offsetAt(safeElapsed);
            double[] adjustedQ = RotationUtil.quaternionMultiply(
                    RotationUtil.quaternionFromRotationVector(rotationVector), targetQ);
            double[] adjustedTotalEuler = RotationUtil.unwrapEuler(
                    targetTotalEuler,
                    RotationUtil.bedrockEulerFromQuaternion(adjustedQ));
            rotationOverrides.put(name, animationEuler(name, adjustedTotalEuler));
        }
        return BonePoseCalculator.calculate(bones, request.targetAnimation(),
                targetTime, positionOverrides, rotationOverrides);
    }

    public double targetSampleTime(double elapsed) {
        BedrockAnimationData.Animation target = request.targetAnimation();
        double time = request.targetEntryTime() + elapsed;
        if (target.loop && target.animationLength > 0) {
            time %= target.animationLength;
            if (time < 0) time += target.animationLength;
            return time;
        }
        return Math.max(0, Math.min(target.animationLength, time));
    }

    private void initializeOffsets() {
        double epsilon = Math.min(1.0 / 120.0,
                request.transitionDuration() * 0.25);
        for (BedrockModelData.Bone bone : bones) {
            if (bone == null || bone.name == null) continue;
            String name = bone.name;
            PhysicsBaker.BoneState physical = physicalStates.get(name);
            double[] sourcePosition = physical != null && physical.position != null
                    ? physical.position.clone()
                    : channel(request.sourceAnimation(), name,
                    request.sourceExitTime(), false);
            double[] targetPosition = channel(request.targetAnimation(), name,
                    request.targetEntryTime(), false);
            double[] positionOffset = subtract(sourcePosition, targetPosition);

            double sourcePreviousTime = Math.max(0,
                    request.sourceExitTime() - epsilon);
            double targetNextTime = Math.min(
                    request.targetAnimation().animationLength,
                    request.targetEntryTime() + epsilon);
            double[] sourceChannelPosition = channel(request.sourceAnimation(), name,
                    request.sourceExitTime(), false);
            double[] sourcePrevious = channel(request.sourceAnimation(), name,
                    sourcePreviousTime, false);
            double[] targetNext = channel(request.targetAnimation(), name,
                    targetNextTime, false);
            double sourceSpan = Math.max(1e-9,
                    request.sourceExitTime() - sourcePreviousTime);
            double targetSpan = Math.max(1e-9,
                    targetNextTime - request.targetEntryTime());
            double[] velocityOffset = new double[3];
            PhysicsBaker.BoneState previousPhysical =
                    previousPhysicalStates.get(name);
            double[] livePositionVelocity = physicalVelocity(
                    physical == null ? null : physical.position,
                    previousPhysical == null ? null : previousPhysical.position);
            for (int axis = 0; axis < 3; axis++) {
                double sourceVelocity = livePositionVelocity == null
                        ? (sourceChannelPosition[axis] - sourcePrevious[axis]) / sourceSpan
                        : livePositionVelocity[axis];
                velocityOffset[axis] = sourceVelocity
                        - (targetNext[axis] - targetPosition[axis]) / targetSpan;
            }

            double[] sourceEuler = physical != null && physical.rotation != null
                    ? physical.rotation.clone()
                    : channel(request.sourceAnimation(), name,
                    request.sourceExitTime(), true);
            double[] targetEuler = channel(request.targetAnimation(), name,
                    request.targetEntryTime(), true);
            double[] sourceTotalEuler = totalEuler(name, sourceEuler);
            double[] targetTotalEuler = totalEuler(name, targetEuler);
            double[] sourceQ = RotationUtil.quaternionFromBedrockEuler(
                    sourceTotalEuler[0], sourceTotalEuler[1], sourceTotalEuler[2]);
            double[] targetQ = RotationUtil.quaternionFromBedrockEuler(
                    targetTotalEuler[0], targetTotalEuler[1], targetTotalEuler[2]);
            double[] rotationOffset = RotationUtil.rotationVectorFromQuaternion(
                    RotationUtil.quaternionMultiply(sourceQ,
                            RotationUtil.quaternionInverse(targetQ)));
            double[] sourcePreviousEuler = channel(request.sourceAnimation(), name,
                    sourcePreviousTime, true);
            double[] targetNextEuler = channel(request.targetAnimation(), name,
                    targetNextTime, true);
            double[] sourceChannelEuler = channel(request.sourceAnimation(), name,
                    request.sourceExitTime(), true);
            double[] sourcePreviousTotalEuler = totalEuler(name, sourcePreviousEuler);
            double[] targetNextTotalEuler = totalEuler(name, targetNextEuler);
            double[] sourceChannelTotalEuler = totalEuler(name, sourceChannelEuler);
            double[] sourceAngularVelocity;
            if (physical != null && physical.rotation != null
                    && previousPhysical != null
                    && previousPhysical.rotation != null
                    && physicalFrameSpan > 0) {
                sourceAngularVelocity = quaternionVelocity(
                        quaternion(totalEuler(name, previousPhysical.rotation)),
                        quaternion(totalEuler(name, physical.rotation)),
                        physicalFrameSpan);
            } else {
                sourceAngularVelocity = quaternionVelocity(
                        quaternion(sourcePreviousTotalEuler),
                        quaternion(sourceChannelTotalEuler), sourceSpan);
            }
            double[] targetAngularVelocity = quaternionVelocity(targetQ,
                    quaternion(targetNextTotalEuler), targetSpan);
            double[] angularVelocityOffset = subtract(
                    sourceAngularVelocity, targetAngularVelocity);
            double weight = request.followWeight(name);
            offsets.put(name, new BoneOffset(
                    new InertializedTarget(positionOffset, velocityOffset,
                            request.transitionDuration(), weight),
                    new InertializedTarget(rotationOffset, angularVelocityOffset,
                            request.transitionDuration(), weight)));
        }
    }

    private static void indexStates(PhysicsBaker.BakedFrame frame,
                                    Map<String, PhysicsBaker.BoneState> target) {
        if (frame == null) return;
        for (PhysicsBaker.BoneState state : frame.boneStates) {
            target.put(state.boneName, state);
        }
    }

    private double[] physicalVelocity(double[] current, double[] previous) {
        if (current == null || previous == null || physicalFrameSpan <= 0) return new double[3];
        double[] result = new double[3];
        for (int axis = 0; axis < 3; axis++) {
            result[axis] = (current[axis] - previous[axis]) / physicalFrameSpan;
        }
        return result;
    }

    private static double[] quaternion(double[] euler) {
        return RotationUtil.quaternionFromBedrockEuler(
                euler[0], euler[1], euler[2]);
    }

    private double[] totalEuler(String boneName, double[] animationEuler) {
        BedrockModelData.Bone bone = bonesByName.get(boneName);
        double[] base = bone == null || bone.rotation == null
                ? new double[3] : bone.rotation;
        return new double[]{base[0] + animationEuler[0],
                base[1] + animationEuler[1],
                base[2] + animationEuler[2]};
    }

    private double[] animationEuler(String boneName, double[] totalEuler) {
        BedrockModelData.Bone bone = bonesByName.get(boneName);
        double[] base = bone == null || bone.rotation == null
                ? new double[3] : bone.rotation;
        return new double[]{totalEuler[0] - base[0],
                totalEuler[1] - base[1], totalEuler[2] - base[2]};
    }

    private static double[] channel(BedrockAnimationData.Animation animation,
                                    String boneName, double time,
                                    boolean rotation) {
        BedrockAnimationData.BoneAnimation bone = animation.bones.get(boneName);
        if (bone == null) return new double[]{0, 0, 0};
        BedrockAnimationData.Keyframes keyframes = rotation ? bone.rotation : bone.position;
        return keyframes == null ? new double[]{0, 0, 0}
                : keyframes.evaluate(time).clone();
    }

    private static double[] subtract(double[] left, double[] right) {
        return new double[]{left[0] - right[0], left[1] - right[1],
                left[2] - right[2]};
    }

    private static double[] quaternionVelocity(double[] from, double[] to,
                                               double span) {
        double[] delta = RotationUtil.quaternionMultiply(
                to, RotationUtil.quaternionInverse(from));
        double[] vector = RotationUtil.rotationVectorFromQuaternion(delta);
        double inverseSpan = 1.0 / Math.max(1e-9, span);
        for (int axis = 0; axis < 3; axis++) vector[axis] *= inverseSpan;
        return vector;
    }

    private record BoneOffset(InertializedTarget position,
                              InertializedTarget rotation) {
    }
}
