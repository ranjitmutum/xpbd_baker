package xpbd.baker;

import xpbd.loader.BedrockAnimationData;
import xpbd.loader.BedrockModelData;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/** 导出时间线的接缝诊断，包含层级关系和四位小数量化影响。 */
public final class LoopSeamReport {
    public record Metrics(
            double maximumPositionError, String positionBone,
            double maximumRotationErrorRadians, String rotationBone,
            double maximumLinearVelocityJump, String linearVelocityBone,
            double maximumAngularVelocityJump, String angularVelocityBone,
            double maximumLinearAccelerationJump, String linearAccelerationBone,
            double maximumAngularAccelerationJump, String angularAccelerationBone,
            double peakLinearVelocity, double peakAngularVelocity,
            double peakLinearAcceleration, double peakAngularAcceleration) {
        public static Metrics empty() {
            return new Metrics(0, null, 0, null, 0, null, 0, null,
                    0, null, 0, null, 0, 0, 0, 0);
        }
    }

    private final Metrics local;
    private final Metrics finalWorld;
    private final Metrics driver;
    private final Metrics physicsRelative;
    private final Metrics quantizedLocal;
    private final Metrics quantizedFinalWorld;
    private final boolean correctionApplied;
    private final double correctionWindowRatio;
    private final boolean collisionSafe;
    private final double maximumPenetration;

    private LoopSeamReport(Metrics local, Metrics finalWorld, Metrics driver,
                           Metrics physicsRelative, Metrics quantizedLocal,
                           Metrics quantizedFinalWorld,
                           boolean correctionApplied, double correctionWindowRatio,
                           boolean collisionSafe, double maximumPenetration) {
        this.local = local;
        this.finalWorld = finalWorld;
        this.driver = driver;
        this.physicsRelative = physicsRelative;
        this.quantizedLocal = quantizedLocal;
        this.quantizedFinalWorld = quantizedFinalWorld;
        this.correctionApplied = correctionApplied;
        this.correctionWindowRatio = correctionWindowRatio;
        this.collisionSafe = collisionSafe;
        this.maximumPenetration = maximumPenetration;
    }

    public static LoopSeamReport measure(
            List<PhysicsBaker.BakedFrame> frames,
            List<BedrockModelData.Bone> bones,
            BedrockAnimationData.Animation animation,
            Set<String> physicsBones,
            Set<String> fixedPhysicsBones,
            boolean correctionApplied,
            double correctionWindowRatio,
            boolean collisionSafe,
            double maximumPenetration) {
        if (frames == null || frames.size() < 2) {
            return new LoopSeamReport(Metrics.empty(), Metrics.empty(),
                    Metrics.empty(), Metrics.empty(), Metrics.empty(), Metrics.empty(),
                    correctionApplied, correctionWindowRatio,
                    collisionSafe, maximumPenetration);
        }
        Set<String> measuredBones = measuredSubtree(bones, physicsBones);
        Map<String, BedrockModelData.Bone> byName = indexBones(bones);
        Map<String, String> anchors = findAnchors(measuredBones, byName,
                physicsBones, fixedPhysicsBones);

        List<Map<String, Transform>> local = localTransforms(frames, byName, false);
        List<Map<String, Transform>> quantizedLocal = localTransforms(frames, byName, true);
        List<Map<String, Transform>> world = worldTransforms(frames, bones,
                animation, false);
        List<Map<String, Transform>> quantizedWorld = worldTransforms(frames, bones,
                animation, true);
        List<Map<String, Transform>> driver = driverTransforms(frames, bones, animation);
        List<Map<String, Transform>> relative = relativeTransforms(world, anchors);
        return new LoopSeamReport(
                measureTransforms(local, physicsBones, frames),
                measureTransforms(world, measuredBones, frames),
                measureTransforms(driver, measuredBones, frames),
                measureTransforms(relative, relativeNames(anchors), frames),
                measureTransforms(quantizedLocal, physicsBones, frames),
                measureTransforms(quantizedWorld, measuredBones, frames),
                correctionApplied, correctionWindowRatio,
                collisionSafe, maximumPenetration);
    }

    public boolean passes(BoneMapper.PhysicsGroupConfig config) {
        Metrics continuity = config.loopSeamStrategy
                == BoneMapper.LoopSeamStrategy.VISUAL_SUBTREE
                ? finalWorld : physicsRelative;
        double linearLimit = Math.max(config.loopSeamMinimumLinearVelocityTolerance,
                continuity.peakLinearVelocity * config.loopSeamRelativeVelocityTolerance);
        double angularLimit = Math.max(config.loopSeamMinimumAngularVelocityTolerance,
                continuity.peakAngularVelocity * config.loopSeamRelativeVelocityTolerance);
        boolean c0 = quantizedLocal.maximumPositionError == 0
                && quantizedLocal.maximumRotationErrorRadians <= 1e-5
                && quantizedFinalWorld.maximumPositionError <= 1e-4
                && quantizedFinalWorld.maximumRotationErrorRadians <= 1e-5;
        boolean c1 = continuity.maximumLinearVelocityJump <= linearLimit
                && continuity.maximumAngularVelocityJump <= angularLimit;
        boolean c2 = true;
        if (config.loopSeamMatchAcceleration) {
            double linearAccelerationLimit = Math.max(
                    config.loopSeamMinimumLinearVelocityTolerance,
                    continuity.peakLinearAcceleration
                            * config.loopSeamRelativeAccelerationTolerance);
            double angularAccelerationLimit = Math.max(
                    config.loopSeamMinimumAngularVelocityTolerance,
                    continuity.peakAngularAcceleration
                            * config.loopSeamRelativeAccelerationTolerance);
            c2 = continuity.maximumLinearAccelerationJump <= linearAccelerationLimit
                    && continuity.maximumAngularAccelerationJump
                    <= angularAccelerationLimit;
        }
        return c0 && c1 && c2 && collisionSafe
                && maximumPenetration <= config.rigidBodyMaximumSafePenetration;
    }

    public double qualityScore(BoneMapper.PhysicsGroupConfig config) {
        Metrics continuity = config.loopSeamStrategy
                == BoneMapper.LoopSeamStrategy.VISUAL_SUBTREE
                ? finalWorld : physicsRelative;
        double linearLimit = Math.max(config.loopSeamMinimumLinearVelocityTolerance,
                continuity.peakLinearVelocity * config.loopSeamRelativeVelocityTolerance);
        double angularLimit = Math.max(config.loopSeamMinimumAngularVelocityTolerance,
                continuity.peakAngularVelocity * config.loopSeamRelativeVelocityTolerance);
        double score = Math.max(
                ratio(quantizedLocal.maximumPositionError, 1e-12),
                ratio(quantizedLocal.maximumRotationErrorRadians, 1e-5));
        score = Math.max(score,
                ratio(quantizedFinalWorld.maximumPositionError, 1e-4));
        score = Math.max(score,
                ratio(quantizedFinalWorld.maximumRotationErrorRadians, 1e-5));
        score = Math.max(score, ratio(continuity.maximumLinearVelocityJump,
                linearLimit));
        score = Math.max(score, ratio(continuity.maximumAngularVelocityJump,
                angularLimit));
        if (config.loopSeamMatchAcceleration) {
            score = Math.max(score, ratio(continuity.maximumLinearAccelerationJump,
                    Math.max(config.loopSeamMinimumLinearVelocityTolerance,
                            continuity.peakLinearAcceleration
                                    * config.loopSeamRelativeAccelerationTolerance)));
            score = Math.max(score, ratio(continuity.maximumAngularAccelerationJump,
                    Math.max(config.loopSeamMinimumAngularVelocityTolerance,
                            continuity.peakAngularAcceleration
                                    * config.loopSeamRelativeAccelerationTolerance)));
        }
        if (!collisionSafe) return Double.POSITIVE_INFINITY;
        return Math.max(score, ratio(maximumPenetration,
                config.rigidBodyMaximumSafePenetration));
    }

    public Metrics local() { return local; }
    public Metrics finalWorld() { return finalWorld; }
    public Metrics driver() { return driver; }
    public Metrics physicsRelative() { return physicsRelative; }
    public Metrics quantizedLocal() { return quantizedLocal; }
    public Metrics quantizedFinalWorld() { return quantizedFinalWorld; }
    public boolean correctionApplied() { return correctionApplied; }
    public double correctionWindowRatio() { return correctionWindowRatio; }
    public boolean collisionSafe() { return collisionSafe; }
    public double maximumPenetration() { return maximumPenetration; }

    private static double ratio(double value, double limit) {
        if (limit == Double.POSITIVE_INFINITY) return 0;
        if (!(limit > 0)) return value == 0 ? 0 : Double.POSITIVE_INFINITY;
        return value / limit;
    }

    private static Metrics measureTransforms(List<Map<String, Transform>> samples,
                                             Set<String> names,
                                             List<PhysicsBaker.BakedFrame> frames) {
        if (samples.size() < 2 || names.isEmpty()) return Metrics.empty();
        int last = samples.size() - 1;
        MetricsAccumulator result = new MetricsAccumulator();
        for (String name : names) {
            Transform first = samples.get(0).get(name);
            Transform end = samples.get(last).get(name);
            if (first == null || end == null) continue;
            result.position(distance(first.position, end.position), name);
            result.rotation(rotationDistance(first.rotation, end.rotation), name);

            double[] outVelocity = velocity(samples, frames, name, 0, 1);
            double[] inVelocity = velocity(samples, frames, name, last - 1, last);
            double[] outAngular = angularVelocity(samples, frames, name, 0, 1);
            double[] inAngular = angularVelocity(samples, frames, name, last - 1, last);
            result.linearVelocity(distance(outVelocity, inVelocity), name);
            result.angularVelocity(distance(outAngular, inAngular), name);

            if (samples.size() >= 3) {
                double[] outAcceleration = acceleration(samples, frames, name, 0, 1, 2, false);
                double[] inAcceleration = acceleration(samples, frames, name,
                        last - 2, last - 1, last, false);
                double[] outAngularAcceleration = acceleration(samples, frames, name,
                        0, 1, 2, true);
                double[] inAngularAcceleration = acceleration(samples, frames, name,
                        last - 2, last - 1, last, true);
                result.linearAcceleration(distance(outAcceleration, inAcceleration), name);
                result.angularAcceleration(distance(outAngularAcceleration,
                        inAngularAcceleration), name);
            }
            for (int i = 1; i < samples.size(); i++) {
                result.peakLinearVelocity = Math.max(result.peakLinearVelocity,
                        length(velocity(samples, frames, name, i - 1, i)));
                result.peakAngularVelocity = Math.max(result.peakAngularVelocity,
                        length(angularVelocity(samples, frames, name, i - 1, i)));
            }
            for (int i = 2; i < samples.size(); i++) {
                result.peakLinearAcceleration = Math.max(result.peakLinearAcceleration,
                        length(acceleration(samples, frames, name, i - 2, i - 1, i, false)));
                result.peakAngularAcceleration = Math.max(result.peakAngularAcceleration,
                        length(acceleration(samples, frames, name, i - 2, i - 1, i, true)));
            }
        }
        return result.build();
    }

    private static List<Map<String, Transform>> localTransforms(
            List<PhysicsBaker.BakedFrame> frames,
            Map<String, BedrockModelData.Bone> bones, boolean quantize) {
        List<Map<String, Transform>> result = new ArrayList<>(frames.size());
        for (PhysicsBaker.BakedFrame frame : frames) {
            Map<String, Transform> values = new HashMap<>();
            for (PhysicsBaker.BoneState state : frame.boneStates) {
                BedrockModelData.Bone bone = bones.get(state.boneName);
                if (bone == null || state.position == null || state.rotation == null) continue;
                double[] position = values(state.position, quantize);
                double[] rotation = values(state.rotation, quantize);
                values.put(state.boneName, new Transform(position,
                        RotationUtil.quaternionFromBedrockEuler(
                                bone.rotation[0] + rotation[0],
                                bone.rotation[1] + rotation[1],
                                bone.rotation[2] + rotation[2])));
            }
            result.add(values);
        }
        return result;
    }

    private static List<Map<String, Transform>> worldTransforms(
            List<PhysicsBaker.BakedFrame> frames, List<BedrockModelData.Bone> bones,
            BedrockAnimationData.Animation animation, boolean quantize) {
        List<Map<String, Transform>> result = new ArrayList<>(frames.size());
        for (PhysicsBaker.BakedFrame frame : frames) {
            Map<String, double[]> positions = new HashMap<>();
            Map<String, double[]> rotations = new HashMap<>();
            for (PhysicsBaker.BoneState state : frame.boneStates) {
                if (state.position != null) positions.put(state.boneName,
                        values(state.position, quantize));
                if (state.rotation != null) rotations.put(state.boneName,
                        values(state.rotation, quantize));
            }
            result.add(transforms(BonePoseCalculator.calculate(bones, animation,
                    frame.time, positions, rotations)));
        }
        return result;
    }

    private static List<Map<String, Transform>> driverTransforms(
            List<PhysicsBaker.BakedFrame> frames, List<BedrockModelData.Bone> bones,
            BedrockAnimationData.Animation animation) {
        List<Map<String, Transform>> result = new ArrayList<>(frames.size());
        for (PhysicsBaker.BakedFrame frame : frames) {
            result.add(transforms(BonePoseCalculator.calculate(
                    bones, animation, frame.time)));
        }
        return result;
    }

    private static Map<String, Transform> transforms(
            Map<String, BonePoseCalculator.Pose> poses) {
        Map<String, Transform> result = new HashMap<>();
        for (Map.Entry<String, BonePoseCalculator.Pose> entry : poses.entrySet()) {
            result.put(entry.getKey(), new Transform(entry.getValue().worldPosition,
                    entry.getValue().worldRotation));
        }
        return result;
    }

    private static List<Map<String, Transform>> relativeTransforms(
            List<Map<String, Transform>> world, Map<String, String> anchors) {
        List<Map<String, Transform>> result = new ArrayList<>(world.size());
        for (Map<String, Transform> sample : world) {
            Map<String, Transform> relative = new HashMap<>();
            for (Map.Entry<String, String> entry : anchors.entrySet()) {
                if (entry.getKey().equals(entry.getValue())) continue;
                Transform value = sample.get(entry.getKey());
                Transform anchor = sample.get(entry.getValue());
                if (value == null || anchor == null) continue;
                double[] inverse = RotationUtil.quaternionInverse(anchor.rotation);
                relative.put(entry.getKey(), new Transform(
                        RotationUtil.rotateVector(inverse,
                                subtract(value.position, anchor.position)),
                        RotationUtil.quaternionMultiply(inverse, value.rotation)));
            }
            result.add(relative);
        }
        return result;
    }

    private static Set<String> measuredSubtree(List<BedrockModelData.Bone> bones,
                                               Set<String> physicsBones) {
        Set<String> roots = new HashSet<>();
        Map<String, BedrockModelData.Bone> byName = indexBones(bones);
        for (String name : physicsBones) {
            BedrockModelData.Bone bone = byName.get(name);
            if (bone != null && (bone.parent == null
                    || !physicsBones.contains(bone.parent))) roots.add(name);
        }
        Set<String> measured = new LinkedHashSet<>();
        boolean changed;
        do {
            changed = false;
            for (BedrockModelData.Bone bone : bones) {
                if (bone == null || bone.name == null) continue;
                if (roots.contains(bone.name)
                        || (bone.parent != null && measured.contains(bone.parent))) {
                    changed |= measured.add(bone.name);
                }
            }
        } while (changed);
        return measured;
    }

    private static Map<String, String> findAnchors(
            Set<String> names, Map<String, BedrockModelData.Bone> bones,
            Set<String> physicsBones, Set<String> fixedPhysicsBones) {
        Map<String, String> result = new HashMap<>();
        for (String name : names) {
            BedrockModelData.Bone current = bones.get(name);
            while (current != null) {
                if (physicsBones.contains(current.name)
                        && fixedPhysicsBones.contains(current.name)) {
                    result.put(name, current.name);
                    break;
                }
                current = current.parent == null ? null : bones.get(current.parent);
            }
        }
        return result;
    }

    private static Set<String> relativeNames(Map<String, String> anchors) {
        Set<String> names = new HashSet<>(anchors.keySet());
        names.removeIf(name -> name.equals(anchors.get(name)));
        return names;
    }

    private static Map<String, BedrockModelData.Bone> indexBones(
            List<BedrockModelData.Bone> bones) {
        Map<String, BedrockModelData.Bone> result = new HashMap<>();
        for (BedrockModelData.Bone bone : bones) {
            if (bone != null && bone.name != null) result.put(bone.name, bone);
        }
        return result;
    }

    private static double[] velocity(List<Map<String, Transform>> samples,
                                     List<PhysicsBaker.BakedFrame> frames,
                                     String name, int from, int to) {
        Transform a = samples.get(from).get(name);
        Transform b = samples.get(to).get(name);
        double dt = timeStep(frames, from, to);
        if (a == null || b == null || !(dt > 0)) return new double[3];
        return scale(subtract(b.position, a.position), 1 / dt);
    }

    private static double[] angularVelocity(List<Map<String, Transform>> samples,
                                            List<PhysicsBaker.BakedFrame> frames,
                                            String name, int from, int to) {
        Transform a = samples.get(from).get(name);
        Transform b = samples.get(to).get(name);
        double dt = timeStep(frames, from, to);
        if (a == null || b == null || !(dt > 0)) return new double[3];
        return scale(RotationUtil.rotationVectorFromQuaternion(
                RotationUtil.quaternionMultiply(
                        RotationUtil.quaternionInverse(a.rotation), b.rotation)), 1 / dt);
    }

    private static double timeStep(List<PhysicsBaker.BakedFrame> frames,
                                   int from, int to) {
        return frames.get(to).time - frames.get(from).time;
    }

    private static double[] acceleration(List<Map<String, Transform>> samples,
                                         List<PhysicsBaker.BakedFrame> frames,
                                         String name, int a, int b, int c,
                                         boolean angular) {
        double[] first = angular ? angularVelocity(samples, frames, name, a, b)
                : velocity(samples, frames, name, a, b);
        double[] second = angular ? angularVelocity(samples, frames, name, b, c)
                : velocity(samples, frames, name, b, c);
        double span = 0.5 * timeStep(frames, a, c);
        return span > 0 ? scale(subtract(second, first), 1 / span) : new double[3];
    }

    private static double rotationDistance(double[] a, double[] b) {
        return length(RotationUtil.rotationVectorFromQuaternion(
                RotationUtil.quaternionMultiply(RotationUtil.quaternionInverse(a), b)));
    }

    private static double[] values(double[] source, boolean quantize) {
        double[] result = source.clone();
        if (quantize) {
            for (int i = 0; i < result.length; i++) {
                result[i] = Math.round(result[i] * 10_000.0) / 10_000.0;
            }
        }
        return result;
    }

    private static double distance(double[] a, double[] b) {
        return length(subtract(a, b));
    }

    private static double length(double[] value) {
        return Math.sqrt(value[0] * value[0] + value[1] * value[1]
                + value[2] * value[2]);
    }

    private static double[] subtract(double[] a, double[] b) {
        return new double[]{a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    private static double[] scale(double[] value, double factor) {
        return new double[]{value[0] * factor, value[1] * factor,
                value[2] * factor};
    }

    private record Transform(double[] position, double[] rotation) {
    }

    private static final class MetricsAccumulator {
        double position;
        double rotation;
        double linearVelocity;
        double angularVelocity;
        double linearAcceleration;
        double angularAcceleration;
        String positionBone;
        String rotationBone;
        String linearVelocityBone;
        String angularVelocityBone;
        String linearAccelerationBone;
        String angularAccelerationBone;
        double peakLinearVelocity;
        double peakAngularVelocity;
        double peakLinearAcceleration;
        double peakAngularAcceleration;

        void position(double value, String bone) { if (value > position) { position = value; positionBone = bone; } }
        void rotation(double value, String bone) { if (value > rotation) { rotation = value; rotationBone = bone; } }
        void linearVelocity(double value, String bone) { if (value > linearVelocity) { linearVelocity = value; linearVelocityBone = bone; } }
        void angularVelocity(double value, String bone) { if (value > angularVelocity) { angularVelocity = value; angularVelocityBone = bone; } }
        void linearAcceleration(double value, String bone) { if (value > linearAcceleration) { linearAcceleration = value; linearAccelerationBone = bone; } }
        void angularAcceleration(double value, String bone) { if (value > angularAcceleration) { angularAcceleration = value; angularAccelerationBone = bone; } }

        Metrics build() {
            return new Metrics(position, positionBone, rotation, rotationBone,
                    linearVelocity, linearVelocityBone, angularVelocity,
                    angularVelocityBone, linearAcceleration, linearAccelerationBone,
                    angularAcceleration, angularAccelerationBone, peakLinearVelocity,
                    peakAngularVelocity, peakLinearAcceleration, peakAngularAcceleration);
        }
    }
}
