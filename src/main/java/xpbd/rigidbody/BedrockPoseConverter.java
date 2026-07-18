package xpbd.rigidbody;

import xpbd.baker.BonePoseCalculator;
import xpbd.baker.BedrockTransformResolver;
import xpbd.baker.RotationUtil;
import xpbd.loader.BedrockModelData;

import java.util.Arrays;
import java.util.Objects;

/** 将刚体的世界枢轴姿态转换回 Bedrock 局部通道。 */
public final class BedrockPoseConverter {
    private BedrockPoseConverter() {
    }

    public record LocalChannels(double[] position, double[] rotation) {
        public LocalChannels {
            position = position.clone();
            rotation = rotation.clone();
        }

        @Override
        public double[] position() {
            return position.clone();
        }

        @Override
        public double[] rotation() {
            return rotation.clone();
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (!(o instanceof LocalChannels that)) return false;
            return Arrays.equals(position, that.position) &&
                   Arrays.equals(rotation, that.rotation);
        }

        @Override
        public int hashCode() {
            return 31 * Arrays.hashCode(position) + Arrays.hashCode(rotation);
        }

        @Override
        public String toString() {
            return "LocalChannels{" +
                   "position=" + Arrays.toString(position) +
                   ", rotation=" + Arrays.toString(rotation) +
                   '}';
        }
    }

    public static RigidBodyBackend.Transform fromPose(
            BonePoseCalculator.Pose pose, double unitScale) {
        Objects.requireNonNull(pose, "pose");
        requireUnitScale(unitScale);
        return new RigidBodyBackend.Transform(new double[]{
                pose.worldPosition[0] * unitScale,
                pose.worldPosition[1] * unitScale,
                pose.worldPosition[2] * unitScale
        }, pose.worldRotation);
    }

    /**
     * @param parentBone 根骨骼时为 null
     * @param parentWorldPivotTransform 根骨骼时为 null；否则为父骨骼的世界枢轴变换
     */
    public static LocalChannels toLocalChannels(
            BedrockModelData.Bone bone,
            RigidBodyBackend.Transform worldPivotTransform,
            BedrockModelData.Bone parentBone,
            RigidBodyBackend.Transform parentWorldPivotTransform,
            double unitScale) {
        Objects.requireNonNull(bone, "bone");
        Objects.requireNonNull(worldPivotTransform, "worldPivotTransform");
        requireUnitScale(unitScale);
        if ((parentBone == null) != (parentWorldPivotTransform == null)) {
            throw new IllegalArgumentException(
                    "parent bone and parent world transform must be supplied together");
        }

        double[] parentRotation = parentWorldPivotTransform == null
                ? new double[]{0, 0, 0, 1}
                : parentWorldPivotTransform.rotation();
        double[] parentModelTranslation = new double[]{0, 0, 0};
        if (parentBone != null) {
            Objects.requireNonNull(parentWorldPivotTransform, "parentWorldPivotTransform");
            double[] parentPivot = BedrockTransformResolver.convertBedrockVector(
                    parentBone.pivot);
            double[] rotatedParentPivot = RotationUtil.rotateVector(parentRotation,
                    new double[]{parentPivot[0] * unitScale,
                            parentPivot[1] * unitScale,
                            parentPivot[2] * unitScale});
            double[] parentWorldPosition = parentWorldPivotTransform.translation();
            parentModelTranslation[0] = parentWorldPosition[0] - rotatedParentPivot[0];
            parentModelTranslation[1] = parentWorldPosition[1] - rotatedParentPivot[1];
            parentModelTranslation[2] = parentWorldPosition[2] - rotatedParentPivot[2];
        }

        double[] worldPosition = worldPivotTransform.translation();
        double[] relativePosition = new double[]{
                worldPosition[0] - parentModelTranslation[0],
                worldPosition[1] - parentModelTranslation[1],
                worldPosition[2] - parentModelTranslation[2]
        };
        double[] localPivotPosition = RotationUtil.rotateVector(
                RotationUtil.quaternionInverse(parentRotation), relativePosition);
        double[] bonePivot = BedrockTransformResolver.convertBedrockVector(bone.pivot);
        double[] mappedAnimationPosition = new double[]{
                localPivotPosition[0] / unitScale - bonePivot[0],
                localPivotPosition[1] / unitScale - bonePivot[1],
                localPivotPosition[2] / unitScale - bonePivot[2]
        };
        double[] animationPosition = BedrockTransformResolver.convertBedrockVector(
                mappedAnimationPosition);

        double[] localRotation = RotationUtil.quaternionMultiply(
                RotationUtil.quaternionInverse(parentRotation),
                worldPivotTransform.rotation());
        double[] totalLocalEuler = RotationUtil.bedrockEulerFromQuaternion(localRotation);
        double[] animationRotation = new double[]{
                wrapDegrees(totalLocalEuler[0] - bone.rotation[0]),
                wrapDegrees(totalLocalEuler[1] - bone.rotation[1]),
                wrapDegrees(totalLocalEuler[2] - bone.rotation[2])
        };
        return new LocalChannels(animationPosition, animationRotation);
    }

    private static double wrapDegrees(double value) {
        double wrapped = value % 360.0;
        if (wrapped > 180) wrapped -= 360;
        if (wrapped < -180) wrapped += 360;
        return wrapped;
    }

    private static void requireUnitScale(double unitScale) {
        if (!Double.isFinite(unitScale) || !(unitScale > 0)) {
            throw new IllegalArgumentException("unit scale must be finite and greater than zero");
        }
    }
}
