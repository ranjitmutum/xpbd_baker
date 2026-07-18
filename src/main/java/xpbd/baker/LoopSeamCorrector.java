package xpbd.baker;

import xpbd.loader.BedrockModelData;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Comparator;

/** 对可导出的局部通道应用端点受约束的五次修正。 */
public final class LoopSeamCorrector {
    private LoopSeamCorrector() {
    }

    public record Result(List<PhysicsBaker.BakedFrame> frames,
                         int windowStartIndex, double windowRatio) {
        public Result {
            frames = List.copyOf(frames);
        }
    }

    public static Result correctCopy(
            List<PhysicsBaker.BakedFrame> source,
            Map<String, BedrockModelData.Bone> bonesByName,
            Set<String> correctedBones,
            double windowRatio,
            boolean matchAcceleration) {
        if (source == null || source.size() < 2) {
            throw new IllegalArgumentException("at least two loop frames are required");
        }
        if (!Double.isFinite(windowRatio) || windowRatio <= 0 || windowRatio > 1) {
            throw new IllegalArgumentException("window ratio must be in (0, 1]");
        }
        List<PhysicsBaker.BakedFrame> frames = deepCopy(source);
        int last = frames.size() - 1;
        int desiredIntervals = (int) Math.ceil(last * windowRatio);
        int minimumIntervals = Math.min(8, last);
        int windowIntervals = Math.min(last,
                Math.max(minimumIntervals, desiredIntervals));
        int start = last - windowIntervals;
        double duration = frames.get(last).time - frames.get(start).time;
        if (!(duration > 0) || !Double.isFinite(duration)) {
            throw new IllegalArgumentException("correction window must have positive duration");
        }

        for (String boneName : correctedBones) {
            BedrockModelData.Bone bone = bonesByName.get(boneName);
            if (bone == null) continue;
            correctPosition(frames, boneName, start, duration, matchAcceleration);
            correctRotation(frames, bone, start, duration, matchAcceleration);
        }
        canonicalizeEndpoint(frames);
        return new Result(frames, start, (double) windowIntervals / last);
    }

    public static Result correctHierarchyCopy(
            List<PhysicsBaker.BakedFrame> source,
            List<BedrockModelData.Bone> bones,
            xpbd.loader.BedrockAnimationData.Animation animation,
            Set<String> physicsBones,
            Set<String> fixedPhysicsBones,
            BoneMapper.LoopSeamStrategy strategy,
            double windowRatio,
            boolean matchAcceleration) {
        if (source == null || source.size() < 3) {
            throw new IllegalArgumentException("at least three loop frames are required");
        }
        List<PhysicsBaker.BakedFrame> frames = deepCopy(source);
        int last = frames.size() - 1;
        int intervals = Math.min(last, Math.max(Math.min(8, last),
                (int) Math.ceil(last * windowRatio)));
        int start = last - intervals;
        double duration = frames.get(last).time - frames.get(start).time;
        Map<String, BedrockModelData.Bone> byName = new HashMap<>();
        for (BedrockModelData.Bone bone : bones) {
            if (bone != null && bone.name != null) byName.put(bone.name, bone);
        }
        Set<String> corrected = new HashSet<>();
        for (String name : physicsBones) {
            if (strategy == BoneMapper.LoopSeamStrategy.VISUAL_SUBTREE
                    || !fixedPhysicsBones.contains(name)) corrected.add(name);
        }
        List<String> ordered = new ArrayList<>(corrected);
        ordered.sort(Comparator.comparingInt(name -> hierarchyDepth(name, byName)));

        List<Map<String, ChannelTarget>> targets = new ArrayList<>(3);
        int[] tailIndices = {last - 2, last - 1, last};
        Map<String, DesiredTransform[]> desired = desiredTransforms(frames, bones,
                animation, ordered, byName, physicsBones, fixedPhysicsBones, strategy,
                matchAcceleration);
        for (int slot = 0; slot < 3; slot++) {
            int frameIndex = tailIndices[slot];
            Map<String, double[]> positionOverrides = overrides(frames.get(frameIndex), true);
            Map<String, double[]> rotationOverrides = overrides(frames.get(frameIndex), false);
            Map<String, ChannelTarget> slotTargets = new HashMap<>();
            targets.add(slotTargets);
            for (String name : ordered) {
                BedrockModelData.Bone bone = byName.get(name);
                DesiredTransform target = desired.get(name)[slot];
                Map<String, BonePoseCalculator.Pose> poses = BonePoseCalculator.calculate(
                        bones, animation, frames.get(frameIndex).time,
                        positionOverrides, rotationOverrides);
                BonePoseCalculator.Pose parent = bone.parent == null
                        ? null : poses.get(bone.parent);
                ChannelTarget channel = solveLocalChannel(bone, parent, target,
                        frames.get(frameIndex).getBoneState(name).rotation);
                positionOverrides.put(name, channel.position);
                rotationOverrides.put(name, channel.rotation);
                slotTargets.put(name, channel);
            }
        }

        double previousU = (frames.get(last - 2).time - frames.get(start).time) / duration;
        double penultimateU = (frames.get(last - 1).time - frames.get(start).time) / duration;
        for (String name : ordered) {
            BedrockModelData.Bone bone = byName.get(name);
            double[][] positionCoefficients = new double[3][3];
            double[][] rotationCoefficients = new double[3][3];
            for (int axis = 0; axis < 3; axis++) {
                double p2 = targets.get(0).get(name).position[axis]
                        - frames.get(last - 2).getBoneState(name).position[axis];
                double p1 = targets.get(1).get(name).position[axis]
                        - frames.get(last - 1).getBoneState(name).position[axis];
                double p0 = targets.get(2).get(name).position[axis]
                        - frames.get(last).getBoneState(name).position[axis];
                positionCoefficients[axis] = sampledValueCoefficients(
                        p0, p1, p2, penultimateU, previousU);
            }
            double[][] correctionVectors = new double[3][];
            for (int slot = 0; slot < 3; slot++) {
                double[] originalQ = totalQuaternion(bone,
                        frames.get(tailIndices[slot]).getBoneState(name).rotation);
                double[] targetQ = totalQuaternion(bone,
                        targets.get(slot).get(name).rotation);
                correctionVectors[slot] = correctionVector(originalQ, targetQ);
            }
            for (int axis = 0; axis < 3; axis++) {
                rotationCoefficients[axis] = sampledValueCoefficients(
                        correctionVectors[2][axis], correctionVectors[1][axis],
                        correctionVectors[0][axis], penultimateU, previousU);
            }
            double[] previousEuler = null;
            for (int index = start; index <= last; index++) {
                double u = normalizedTime(frames, start, index, duration);
                PhysicsBaker.BoneState state = frames.get(index).getBoneState(name);
                for (int axis = 0; axis < 3; axis++) {
                    state.position[axis] += evaluate(positionCoefficients[axis], u);
                }
                double[] originalQ = totalQuaternion(bone, state.rotation);
                double[] correction = new double[3];
                for (int axis = 0; axis < 3; axis++) {
                    correction[axis] = evaluate(rotationCoefficients[axis], u);
                }
                double[] totalEuler = RotationUtil.bedrockEulerFromQuaternion(
                        RotationUtil.quaternionMultiply(originalQ,
                                RotationUtil.quaternionFromRotationVector(correction)));
                double[] guide = previousEuler == null ? new double[]{
                        bone.rotation[0] + state.rotation[0],
                        bone.rotation[1] + state.rotation[1],
                        bone.rotation[2] + state.rotation[2]} : previousEuler;
                totalEuler = RotationUtil.unwrapEuler(guide, totalEuler);
                for (int axis = 0; axis < 3; axis++) {
                    state.rotation[axis] = totalEuler[axis] - bone.rotation[axis];
                }
                previousEuler = totalEuler;
            }
        }
        canonicalizeEndpoint(frames);
        return new Result(frames, start, (double) intervals / last);
    }

    private static Map<String, DesiredTransform[]> desiredTransforms(
            List<PhysicsBaker.BakedFrame> frames, List<BedrockModelData.Bone> bones,
            xpbd.loader.BedrockAnimationData.Animation animation,
            List<String> corrected, Map<String, BedrockModelData.Bone> byName,
            Set<String> physicsBones, Set<String> fixedBones,
            BoneMapper.LoopSeamStrategy strategy, boolean matchAcceleration) {
        int last = frames.size() - 1;
        List<Map<String, BonePoseCalculator.Pose>> poses = new ArrayList<>();
        for (PhysicsBaker.BakedFrame frame : frames) {
            poses.add(BonePoseCalculator.calculate(bones, animation, frame.time,
                    overrides(frame, true), overrides(frame, false)));
        }
        double lastStep = frames.get(last).time - frames.get(last - 1).time;
        double previousStep = frames.get(last - 1).time - frames.get(last - 2).time;
        double firstStep = frames.get(1).time - frames.get(0).time;
        double nextStep = frames.get(2).time - frames.get(1).time;
        Map<String, DesiredTransform[]> result = new HashMap<>();
        for (String name : corrected) {
            String anchor = strategy == BoneMapper.LoopSeamStrategy.PHYSICS_RELATIVE
                    ? fixedAnchor(name, byName, physicsBones, fixedBones) : null;
            DesiredTransform first = transform(poses.get(0), name, anchor);
            DesiredTransform second = transform(poses.get(1), name, anchor);
            DesiredTransform third = transform(poses.get(2), name, anchor);
            double[] velocity = scale(subtract(second.position, first.position),
                    1 / firstStep);
            double[] nextVelocity = scale(subtract(third.position, second.position),
                    1 / nextStep);
            double[] acceleration = matchAcceleration
                    ? scale(subtract(nextVelocity, velocity),
                    1 / (0.5 * (firstStep + nextStep))) : new double[3];
            double[] angular = angularVelocity(first.rotation, second.rotation, firstStep);
            double[] nextAngular = angularVelocity(second.rotation, third.rotation, nextStep);
            double[] angularAcceleration = matchAcceleration
                    ? scale(subtract(nextAngular, angular),
                    1 / (0.5 * (firstStep + nextStep))) : new double[3];
            double span = 0.5 * (lastStep + previousStep);
            DesiredTransform end = first;
            DesiredTransform penultimate = new DesiredTransform(
                    subtract(end.position, scale(velocity, lastStep)),
                    RotationUtil.quaternionMultiply(end.rotation,
                            RotationUtil.quaternionFromRotationVector(
                                    scale(angular, -lastStep))));
            double[] previousVelocity = subtract(velocity,
                    scale(acceleration, span));
            double[] previousAngular = subtract(angular,
                    scale(angularAcceleration, span));
            DesiredTransform previous = new DesiredTransform(
                    subtract(penultimate.position,
                            scale(previousVelocity, previousStep)),
                    RotationUtil.quaternionMultiply(penultimate.rotation,
                            RotationUtil.quaternionFromRotationVector(
                                    scale(previousAngular, -previousStep))));
            DesiredTransform[] values = {previous, penultimate, end};
            if (anchor != null) {
                for (int slot = 0; slot < 3; slot++) {
                    BonePoseCalculator.Pose anchorPose = poses.get(last - 2 + slot).get(anchor);
                    values[slot] = compose(anchorPose, values[slot]);
                }
            }
            result.put(name, values);
        }
        return result;
    }

    private static DesiredTransform transform(
            Map<String, BonePoseCalculator.Pose> poses, String name, String anchor) {
        BonePoseCalculator.Pose pose = poses.get(name);
        if (anchor == null) return new DesiredTransform(
                pose.worldPosition.clone(), pose.worldRotation.clone());
        BonePoseCalculator.Pose root = poses.get(anchor);
        double[] inverse = RotationUtil.quaternionInverse(root.worldRotation);
        return new DesiredTransform(RotationUtil.rotateVector(inverse,
                subtract(pose.worldPosition, root.worldPosition)),
                RotationUtil.quaternionMultiply(inverse, pose.worldRotation));
    }

    private static DesiredTransform compose(BonePoseCalculator.Pose anchor,
                                            DesiredTransform relative) {
        double[] offset = RotationUtil.rotateVector(anchor.worldRotation,
                relative.position);
        return new DesiredTransform(new double[]{anchor.worldPosition[0] + offset[0],
                anchor.worldPosition[1] + offset[1],
                anchor.worldPosition[2] + offset[2]},
                RotationUtil.quaternionMultiply(anchor.worldRotation, relative.rotation));
    }

    private static ChannelTarget solveLocalChannel(BedrockModelData.Bone bone,
                                                   BonePoseCalculator.Pose parent,
                                                   DesiredTransform desired,
                                                   double[] originalRotation) {
        double[] parentQ = parent == null ? new double[]{0, 0, 0, 1}
                : parent.worldRotation;
        double[] parentTranslation = parent == null ? new double[3]
                : parent.worldTranslation;
        double[] localQ = RotationUtil.quaternionMultiply(
                RotationUtil.quaternionInverse(parentQ), desired.rotation);
        double[] totalEuler = RotationUtil.unwrapEuler(new double[]{
                        bone.rotation[0] + originalRotation[0],
                        bone.rotation[1] + originalRotation[1],
                        bone.rotation[2] + originalRotation[2]},
                RotationUtil.bedrockEulerFromQuaternion(localQ));
        double[] localPivotPosition = RotationUtil.rotateVector(
                RotationUtil.quaternionInverse(parentQ),
                subtract(desired.position, parentTranslation));
        double[] mappedPivot = BedrockTransformResolver.convertBedrockVector(bone.pivot);
        double[] mappedAnimation = subtract(localPivotPosition, mappedPivot);
        double[] position = BedrockTransformResolver.convertBedrockVector(mappedAnimation);
        return new ChannelTarget(position, new double[]{
                totalEuler[0] - bone.rotation[0],
                totalEuler[1] - bone.rotation[1],
                totalEuler[2] - bone.rotation[2]});
    }

    private static Map<String, double[]> overrides(PhysicsBaker.BakedFrame frame,
                                                   boolean position) {
        Map<String, double[]> result = new HashMap<>();
        for (PhysicsBaker.BoneState state : frame.boneStates) {
            double[] value = position ? state.position : state.rotation;
            if (value != null) result.put(state.boneName, value.clone());
        }
        return result;
    }

    private static String fixedAnchor(String name,
                                      Map<String, BedrockModelData.Bone> bones,
                                      Set<String> physicsBones, Set<String> fixedBones) {
        BedrockModelData.Bone current = bones.get(name);
        while (current != null) {
            if (physicsBones.contains(current.name) && fixedBones.contains(current.name)) {
                return current.name;
            }
            current = current.parent == null ? null : bones.get(current.parent);
        }
        return null;
    }

    private static int hierarchyDepth(String name,
                                      Map<String, BedrockModelData.Bone> bones) {
        int depth = 0;
        BedrockModelData.Bone current = bones.get(name);
        while (current != null && current.parent != null) {
            depth++;
            current = bones.get(current.parent);
        }
        return depth;
    }

    private record DesiredTransform(double[] position, double[] rotation) {}
    private record ChannelTarget(double[] position, double[] rotation) {}

    private static void correctPosition(List<PhysicsBaker.BakedFrame> frames,
                                        String boneName, int start,
                                        double duration, boolean matchAcceleration) {
        int last = frames.size() - 1;
        PhysicsBaker.BoneState firstState = frames.get(0).getBoneState(boneName);
        PhysicsBaker.BoneState lastState = frames.get(last).getBoneState(boneName);
        if (firstState == null || lastState == null
                || firstState.position == null || lastState.position == null) return;

        double[] firstVelocity = linearVelocity(frames, boneName, 0, 1, false);
        double[] lastVelocity = linearVelocity(frames, boneName, last - 1, last, false);
        double[] firstAcceleration = matchAcceleration
                ? linearAcceleration(frames, boneName, 0, 1, 2, false)
                : new double[3];
        double[] lastAcceleration = matchAcceleration
                ? linearAcceleration(frames, boneName, last - 2, last - 1, last, false)
                : new double[3];

        double[][] coefficients = new double[3][3];
        double lastStep = frames.get(last).time - frames.get(last - 1).time;
        double previousStep = last >= 2
                ? frames.get(last - 1).time - frames.get(last - 2).time : lastStep;
        double penultimateU = (frames.get(last - 1).time
                - frames.get(start).time) / duration;
        double previousU = last >= 2 ? (frames.get(last - 2).time
                - frames.get(start).time) / duration : 0;
        for (int axis = 0; axis < 3; axis++) {
            double dx = firstState.position[axis] - lastState.position[axis];
            double dv = firstVelocity[axis] - lastVelocity[axis];
            double da = firstAcceleration[axis] - lastAcceleration[axis];
            coefficients[axis] = sampledCoefficients(dx, dv, da,
                    lastStep, previousStep, penultimateU, previousU);
        }
        for (int index = start; index <= last; index++) {
            PhysicsBaker.BoneState state = frames.get(index).getBoneState(boneName);
            if (state == null || state.position == null) continue;
            double u = normalizedTime(frames, start, index, duration);
            for (int axis = 0; axis < 3; axis++) {
                state.position[axis] += evaluate(coefficients[axis], u);
            }
        }
    }

    private static void correctRotation(List<PhysicsBaker.BakedFrame> frames,
                                        BedrockModelData.Bone bone, int start,
                                        double duration, boolean matchAcceleration) {
        String boneName = bone.name;
        int last = frames.size() - 1;
        double[][] quaternions = new double[frames.size()][];
        for (int i = 0; i < frames.size(); i++) {
            PhysicsBaker.BoneState state = frames.get(i).getBoneState(boneName);
            if (state == null || state.rotation == null) return;
            quaternions[i] = totalQuaternion(bone, state.rotation);
            if (i > 0 && quaternionDot(quaternions[i - 1], quaternions[i]) < 0) {
                negate(quaternions[i]);
            }
        }

        double[] firstVelocity = angularVelocity(quaternions[0], quaternions[1],
                frames.get(1).time - frames.get(0).time);
        double[] firstAcceleration = new double[3];
        if (matchAcceleration && frames.size() >= 3) {
            double[] nextVelocity = angularVelocity(quaternions[1], quaternions[2],
                    frames.get(2).time - frames.get(1).time);
            double firstSpan = 0.5 * (frames.get(2).time - frames.get(0).time);
            for (int axis = 0; axis < 3; axis++) {
                firstAcceleration[axis] = (nextVelocity[axis]
                        - firstVelocity[axis]) / firstSpan;
            }
        }

        double lastStep = frames.get(last).time - frames.get(last - 1).time;
        double previousStep = frames.get(last - 1).time
                - frames.get(last - 2).time;
        double accelerationSpan = 0.5 * (lastStep + previousStep);
        double[] desiredLast = quaternions[0];
        double[] desiredPenultimate = RotationUtil.quaternionMultiply(desiredLast,
                RotationUtil.quaternionFromRotationVector(scale(firstVelocity,
                        -lastStep)));
        double[] desiredPreviousVelocity = new double[3];
        for (int axis = 0; axis < 3; axis++) {
            desiredPreviousVelocity[axis] = firstVelocity[axis]
                    - firstAcceleration[axis] * accelerationSpan;
        }
        double[] desiredPrevious = RotationUtil.quaternionMultiply(
                desiredPenultimate, RotationUtil.quaternionFromRotationVector(
                        scale(desiredPreviousVelocity, -previousStep)));
        double[] atLast = correctionVector(quaternions[last], desiredLast);
        double[] atPenultimate = correctionVector(quaternions[last - 1],
                desiredPenultimate);
        double[] atPrevious = correctionVector(quaternions[last - 2],
                desiredPrevious);
        double penultimateU = (frames.get(last - 1).time
                - frames.get(start).time) / duration;
        double previousU = (frames.get(last - 2).time
                - frames.get(start).time) / duration;
        double[][] coefficients = new double[3][3];
        for (int axis = 0; axis < 3; axis++) {
            coefficients[axis] = sampledValueCoefficients(atLast[axis],
                    atPenultimate[axis], atPrevious[axis],
                    penultimateU, previousU);
        }

        double[] previousTotalEuler = null;
        for (int index = start; index <= last; index++) {
            double u = normalizedTime(frames, start, index, duration);
            double[] correction = new double[3];
            for (int axis = 0; axis < 3; axis++) {
                correction[axis] = evaluate(coefficients[axis], u);
            }
            double[] corrected = RotationUtil.quaternionMultiply(quaternions[index],
                    RotationUtil.quaternionFromRotationVector(correction));
            double[] totalEuler = RotationUtil.bedrockEulerFromQuaternion(corrected);
            PhysicsBaker.BoneState state = frames.get(index).getBoneState(boneName);
            double[] originalTotal = new double[]{
                    bone.rotation[0] + state.rotation[0],
                    bone.rotation[1] + state.rotation[1],
                    bone.rotation[2] + state.rotation[2]
            };
            double[] targetTotal = new double[]{
                    bone.rotation[0] + frames.get(0).getBoneState(boneName).rotation[0],
                    bone.rotation[1] + frames.get(0).getBoneState(boneName).rotation[1],
                    bone.rotation[2] + frames.get(0).getBoneState(boneName).rotation[2]
            };
            double guideWeight = u * u * (3 - 2 * u);
            double[] guide = new double[3];
            for (int axis = 0; axis < 3; axis++) {
                guide[axis] = originalTotal[axis] * (1 - guideWeight)
                        + targetTotal[axis] * guideWeight;
            }
            totalEuler = RotationUtil.unwrapEuler(
                    previousTotalEuler == null ? guide : previousTotalEuler, totalEuler);
            for (int axis = 0; axis < 3; axis++) {
                state.rotation[axis] = totalEuler[axis] - bone.rotation[axis];
            }
            previousTotalEuler = totalEuler;
        }
    }

    private static void canonicalizeEndpoint(List<PhysicsBaker.BakedFrame> frames) {
        PhysicsBaker.BakedFrame first = frames.get(0);
        PhysicsBaker.BakedFrame last = frames.get(frames.size() - 1);
        for (PhysicsBaker.BoneState state : last.boneStates) {
            PhysicsBaker.BoneState target = first.getBoneState(state.boneName);
            if (target == null) continue;
            copy(target.position, state.position);
            copy(target.rotation, state.rotation);
        }
    }

    private static double[] linearVelocity(List<PhysicsBaker.BakedFrame> frames,
                                           String boneName, int from, int to,
                                           boolean world) {
        double dt = frames.get(to).time - frames.get(from).time;
        if (!(dt > 0)) return new double[3];
        double[] a = vector(frames.get(from).getBoneState(boneName), world);
        double[] b = vector(frames.get(to).getBoneState(boneName), world);
        if (a.length != 3 || b.length != 3) return new double[3];
        return new double[]{(b[0] - a[0]) / dt, (b[1] - a[1]) / dt,
                (b[2] - a[2]) / dt};
    }

    private static double[] linearAcceleration(List<PhysicsBaker.BakedFrame> frames,
                                               String boneName, int a, int b, int c,
                                               boolean world) {
        double[] first = linearVelocity(frames, boneName, a, b, world);
        double[] second = linearVelocity(frames, boneName, b, c, world);
        double span = 0.5 * (frames.get(c).time - frames.get(a).time);
        if (!(span > 0)) return new double[3];
        return new double[]{(second[0] - first[0]) / span,
                (second[1] - first[1]) / span,
                (second[2] - first[2]) / span};
    }

    private static double[] vector(PhysicsBaker.BoneState state, boolean world) {
        if (state == null) return new double[0];
        return world ? state.worldPosition : state.position;
    }

    private static double[] angularVelocity(double[] from, double[] to, double dt) {
        if (!(dt > 0)) return new double[3];
        double[] value = RotationUtil.rotationVectorFromQuaternion(
                RotationUtil.quaternionMultiply(RotationUtil.quaternionInverse(from), to));
        return new double[]{value[0] / dt, value[1] / dt, value[2] / dt};
    }

    private static double[] totalQuaternion(BedrockModelData.Bone bone,
                                            double[] animationRotation) {
        return RotationUtil.quaternionFromBedrockEuler(
                bone.rotation[0] + animationRotation[0],
                bone.rotation[1] + animationRotation[1],
                bone.rotation[2] + animationRotation[2]);
    }


    /*
         * 导出动画是离散采样结果，因此要在实际最后两个有限差分区间施加端点约束。
         * u³/u⁴/u⁵ 基函数仍可确保入口处的修正值、速度和加速度均为零。
     */
    private static double[] sampledCoefficients(double dx, double dv, double da,
                                                double lastStep,
                                                double previousStep,
                                                double penultimateU,
                                                double previousU) {
        double accelerationSpan = 0.5 * (lastStep + previousStep);
        double atLast = dx;
        double atPenultimate = atLast - dv * lastStep;
        double atPrevious = atPenultimate
                - previousStep * (dv - da * accelerationSpan);
        return sampledValueCoefficients(atLast, atPenultimate, atPrevious,
                penultimateU, previousU);
    }

    private static double[] sampledValueCoefficients(double atLast,
                                                     double atPenultimate,
                                                     double atPrevious,
                                                     double penultimateU,
                                                     double previousU) {
        double[][] matrix = {
                {1, 1, 1, atLast},
                {cube(penultimateU), fourth(penultimateU), fifth(penultimateU),
                        atPenultimate},
                {cube(previousU), fourth(previousU), fifth(previousU), atPrevious}
        };
        return solve3(matrix);
    }

    private static double[] correctionVector(double[] from, double[] to) {
        return RotationUtil.rotationVectorFromQuaternion(
                RotationUtil.quaternionMultiply(RotationUtil.quaternionInverse(from), to));
    }

    private static double[] scale(double[] value, double factor) {
        return new double[]{value[0] * factor, value[1] * factor,
                value[2] * factor};
    }

    private static double[] subtract(double[] a, double[] b) {
        return new double[]{a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    private static double[] solve3(double[][] matrix) {
        for (int column = 0; column < 3; column++) {
            int pivot = column;
            for (int row = column + 1; row < 3; row++) {
                if (Math.abs(matrix[row][column])
                        > Math.abs(matrix[pivot][column])) pivot = row;
            }
            if (Math.abs(matrix[pivot][column]) < 1e-12) {
                return new double[]{0, 0, matrix[0][3]};
            }
            double[] swap = matrix[column];
            matrix[column] = matrix[pivot];
            matrix[pivot] = swap;
            double scale = matrix[column][column];
            for (int j = column; j < 4; j++) matrix[column][j] /= scale;
            for (int row = 0; row < 3; row++) {
                if (row == column) continue;
                double factor = matrix[row][column];
                for (int j = column; j < 4; j++) {
                    matrix[row][j] -= factor * matrix[column][j];
                }
            }
        }
        return new double[]{matrix[0][3], matrix[1][3], matrix[2][3]};
    }

    private static double cube(double value) {
        return value * value * value;
    }

    private static double fourth(double value) {
        return cube(value) * value;
    }

    private static double fifth(double value) {
        return fourth(value) * value;
    }

    private static double evaluate(double[] coefficients, double u) {
        double u2 = u * u;
        double u3 = u2 * u;
        return coefficients[0] * u3 + coefficients[1] * u3 * u
                + coefficients[2] * u3 * u2;
    }

    private static double normalizedTime(List<PhysicsBaker.BakedFrame> frames,
                                         int start, int index, double duration) {
        return Math.max(0, Math.min(1,
                (frames.get(index).time - frames.get(start).time) / duration));
    }

    private static double quaternionDot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    }

    private static void negate(double[] value) {
        for (int i = 0; i < value.length; i++) value[i] = -value[i];
    }

    private static void copy(double[] source, double[] destination) {
        if (source == null || destination == null) return;
        System.arraycopy(source, 0, destination, 0,
                Math.min(source.length, destination.length));
    }

    private static List<PhysicsBaker.BakedFrame> deepCopy(
            List<PhysicsBaker.BakedFrame> source) {
        List<PhysicsBaker.BakedFrame> result = new ArrayList<>(source.size());
        for (PhysicsBaker.BakedFrame frame : source) {
            List<PhysicsBaker.BoneState> states = new ArrayList<>(frame.boneStates.size());
            for (PhysicsBaker.BoneState state : frame.boneStates) {
                states.add(new PhysicsBaker.BoneState(state.boneName,
                        cloneOrNull(state.position), cloneOrNull(state.rotation),
                        cloneOrNull(state.linearVelocity),
                        cloneOrNull(state.worldPosition)));
            }
            result.add(new PhysicsBaker.BakedFrame(frame.time, states));
        }
        return result;
    }

    private static double[] cloneOrNull(double[] value) {
        return value == null ? null : value.clone();
    }
}
