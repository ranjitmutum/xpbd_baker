package xpbd.baker;

import xpbd.loader.BedrockAnimationData;
import xpbd.loader.BedrockModelData;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/** 在模型/世界空间中计算分层 Bedrock 骨骼枢轴。 */
public final class BonePoseCalculator {
    private BonePoseCalculator() {
    }

    /** 预编译不可变层级查找表与父级优先的遍历顺序。 */
    public static Evaluator compile(List<BedrockModelData.Bone> bones) {
        return new Evaluator(bones);
    }

    public static final class Evaluator {
        private final List<BedrockModelData.Bone> orderedBones;

        private Evaluator(List<BedrockModelData.Bone> bones) {
            Objects.requireNonNull(bones, "bones");
            Map<String, BedrockModelData.Bone> byName = new HashMap<>();
            for (BedrockModelData.Bone bone : bones) {
                if (bone != null && bone.name != null) byName.put(bone.name, bone);
            }
            List<BedrockModelData.Bone> ordered = new ArrayList<>(byName.size());
            Map<String, Integer> states = new HashMap<>();
            for (BedrockModelData.Bone bone : bones) {
                if (bone != null && bone.name != null) {
                    appendTopologically(bone, byName, states, ordered);
                }
            }
            orderedBones = List.copyOf(ordered);
        }

        public Map<String, Pose> calculate(
                BedrockAnimationData.Animation animation, double time) {
            return calculate(animation, time, null, null);
        }

        public Map<String, Pose> calculate(
                BedrockAnimationData.Animation animation, double time,
                Map<String, double[]> positionOverrides,
                Map<String, double[]> rotationOverrides) {
            Map<String, Pose> result = new HashMap<>(
                    Math.max(16, orderedBones.size() * 2));
            for (BedrockModelData.Bone bone : orderedBones) {
                Pose parentPose = bone.parent == null
                        ? null : result.get(bone.parent);
                result.put(bone.name, compose(bone, parentPose, animation, time,
                        positionOverrides, rotationOverrides));
            }
            return result;
        }

        private static void appendTopologically(
                BedrockModelData.Bone bone,
                Map<String, BedrockModelData.Bone> byName,
                Map<String, Integer> states,
                List<BedrockModelData.Bone> ordered) {
            Integer state = states.get(bone.name);
            if (state != null) {
                if (state == 1) {
                    throw new IllegalArgumentException(
                            "Bone hierarchy contains a cycle at " + bone.name);
                }
                return;
            }
            states.put(bone.name, 1);
            if (bone.parent != null) {
                BedrockModelData.Bone parent = byName.get(bone.parent);
                if (parent != null) {
                    appendTopologically(parent, byName, states, ordered);
                }
            }
            states.put(bone.name, 2);
            ordered.add(bone);
        }
    }

    public static Map<String, Pose> calculate(List<BedrockModelData.Bone> bones,
                                               BedrockAnimationData.Animation animation,
                                               double time) {
        return calculate(bones, animation, time, null, null);
    }

    /** 计算姿态，并替换指定骨骼的局部动画通道。 */
    public static Map<String, Pose> calculate(List<BedrockModelData.Bone> bones,
                                               BedrockAnimationData.Animation animation,
                                               double time,
                                               Map<String, double[]> positionOverrides,
                                               Map<String, double[]> rotationOverrides) {
        Map<String, BedrockModelData.Bone> byName = new HashMap<>();
        for (BedrockModelData.Bone bone : bones) {
            if (bone != null && bone.name != null) byName.put(bone.name, bone);
        }
        Map<String, Pose> result = new HashMap<>();
        Set<String> visiting = new HashSet<>();
        for (BedrockModelData.Bone bone : bones) {
            if (bone != null && bone.name != null) {
                resolve(bone, byName, animation, time, positionOverrides,
                        rotationOverrides, result, visiting);
            }
        }
        return result;
    }

    private static Pose resolve(BedrockModelData.Bone bone,
                                Map<String, BedrockModelData.Bone> byName,
                                 BedrockAnimationData.Animation animation,
                                 double time,
                                 Map<String, double[]> positionOverrides,
                                 Map<String, double[]> rotationOverrides,
                                 Map<String, Pose> result,
                                Set<String> visiting) {
        Pose cached = result.get(bone.name);
        if (cached != null) return cached;
        if (!visiting.add(bone.name)) {
            throw new IllegalArgumentException("Bone hierarchy contains a cycle at " + bone.name);
        }

        Pose parentPose = null;
        if (bone.parent != null) {
            BedrockModelData.Bone parent = byName.get(bone.parent);
            if (parent != null) {
                parentPose = resolve(parent, byName, animation, time,
                        positionOverrides, rotationOverrides, result, visiting);
            }
        }

        Pose pose = compose(bone, parentPose, animation, time,
                positionOverrides, rotationOverrides);
        result.put(bone.name, pose);
        visiting.remove(bone.name);
        return pose;
    }

    private static Pose compose(BedrockModelData.Bone bone, Pose parentPose,
                                BedrockAnimationData.Animation animation,
                                double time,
                                Map<String, double[]> positionOverrides,
                                Map<String, double[]> rotationOverrides) {
        double[] animPosition = new double[]{0, 0, 0};
        double[] animRotation = new double[]{0, 0, 0};
        if (animation != null) {
            BedrockAnimationData.BoneAnimation channel = animation.bones.get(bone.name);
            if (channel != null) {
                if (channel.position != null) animPosition = channel.position.evaluate(time).clone();
                if (channel.rotation != null) animRotation = channel.rotation.evaluate(time).clone();
            }
        }
        if (positionOverrides != null && positionOverrides.containsKey(bone.name)) {
            animPosition = positionOverrides.get(bone.name).clone();
        }
        if (rotationOverrides != null && rotationOverrides.containsKey(bone.name)) {
            animRotation = rotationOverrides.get(bone.name).clone();
        }

        double[] totalEuler = new double[]{
                bone.rotation[0] + animRotation[0],
                bone.rotation[1] + animRotation[1],
                bone.rotation[2] + animRotation[2]
        };
        double[] localRotation = RotationUtil.quaternionFromBedrockEuler(
                totalEuler[0], totalEuler[1], totalEuler[2]);
        double[] mappedPivot = BedrockTransformResolver.convertBedrockVector(
                bone.pivot);
        double[] mappedAnimPosition = BedrockTransformResolver.convertBedrockVector(
                animPosition);

        // 局部矩阵 = 平移动画位置 × 平移枢轴 × 旋转 × 反向平移枢轴。
        double[] rotatedNegativePivot = RotationUtil.rotateVector(localRotation,
                new double[]{-mappedPivot[0], -mappedPivot[1], -mappedPivot[2]});
        double[] localTranslation = new double[]{
                mappedAnimPosition[0] + mappedPivot[0] + rotatedNegativePivot[0],
                mappedAnimPosition[1] + mappedPivot[1] + rotatedNegativePivot[1],
                mappedAnimPosition[2] + mappedPivot[2] + rotatedNegativePivot[2]
        };

        double[] worldRotation;
        double[] worldTranslation;
        if (parentPose == null) {
            worldRotation = localRotation;
            worldTranslation = localTranslation;
        } else {
            worldRotation = RotationUtil.quaternionMultiply(
                    parentPose.worldRotation, localRotation);
            double[] translated = RotationUtil.rotateVector(
                    parentPose.worldRotation, localTranslation);
            worldTranslation = new double[]{
                    translated[0] + parentPose.worldTranslation[0],
                    translated[1] + parentPose.worldTranslation[1],
                    translated[2] + parentPose.worldTranslation[2]
            };
        }

        double[] rotatedPivot = RotationUtil.rotateVector(worldRotation, mappedPivot);
        double[] worldPosition = new double[]{
                rotatedPivot[0] + worldTranslation[0],
                rotatedPivot[1] + worldTranslation[1],
                rotatedPivot[2] + worldTranslation[2]
        };
        Pose pose = new Pose(worldPosition, worldRotation, worldTranslation,
                animPosition, totalEuler);
        return pose;
    }

    public static final class Pose {
        public final double[] worldPosition;
        public final double[] worldRotation;
        public final double[] worldTranslation;
        public final double[] animationPosition;
        public final double[] totalLocalEuler;

        private Pose(double[] worldPosition, double[] worldRotation,
                     double[] worldTranslation, double[] animationPosition,
                     double[] totalLocalEuler) {
            this.worldPosition = worldPosition;
            this.worldRotation = worldRotation;
            this.worldTranslation = worldTranslation;
            this.animationPosition = animationPosition;
            this.totalLocalEuler = totalLocalEuler;
        }
    }

}
