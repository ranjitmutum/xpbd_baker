package xpbd.rigidbody;

import xpbd.baker.BonePoseCalculator;
import xpbd.baker.CubeGeometry;
import xpbd.baker.RotationUtil;
import xpbd.loader.BedrockModelData;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

/** 将一个 Bedrock 骨骼及其立方体转换为复合盒体刚体。 */
public final class BedrockRigidBodyCompiler {
    private static final double MIN_HALF_EXTENT = 1e-6;

    private BedrockRigidBodyCompiler() {
    }

    public record Compilation(Optional<RigidBodyBackend.BodyDefinition> body,
                              int sourceCubeCount, int skippedDegenerateCubeCount,
                              String diagnostic) {
        public Compilation {
            body = Objects.requireNonNull(body, "body");
            diagnostic = diagnostic == null ? "" : diagnostic;
        }
    }

    /** 其立方体为复合刚体贡献几何形状的骨骼。 */
    public record CubeSource(BedrockModelData.Bone bone,
                             BonePoseCalculator.Pose pose) {
        public CubeSource {
            Objects.requireNonNull(bone, "bone");
            Objects.requireNonNull(pose, "pose");
        }
    }

    public static Compilation compile(BedrockModelData.Bone bone,
                                      BonePoseCalculator.Pose initialPose,
                                      RigidBodyBackend.MotionType motionType,
                                      double mass, double unitScale) {
        return compile(bone, initialPose, motionType, mass, unitScale,
                0.5, 0, true);
    }

    public static Compilation compile(BedrockModelData.Bone bone,
                                      BonePoseCalculator.Pose initialPose,
                                      RigidBodyBackend.MotionType motionType,
                                      double mass, double unitScale,
                                      double friction, double restitution,
                                      boolean enableCcd) {
        return compileCompound(bone, initialPose,
                List.of(new CubeSource(bone, initialPose)), motionType, mass,
                unitScale, friction, restitution, enableCcd);
    }

    /**
     * 将一个或多个后代组拥有的立方体编译到 {@code bodyBone} 的坐标系中。
     * 后代姿态只在初始化时采样，之后所有贡献立方体会随选定刚体整体运动。
     */
    public static Compilation compileCompound(
            BedrockModelData.Bone bodyBone,
            BonePoseCalculator.Pose bodyPose,
            List<CubeSource> cubeSources,
            RigidBodyBackend.MotionType motionType,
            double mass, double unitScale,
            double friction, double restitution,
            boolean enableCcd) {
        Objects.requireNonNull(bodyBone, "bodyBone");
        Objects.requireNonNull(bodyPose, "bodyPose");
        Objects.requireNonNull(cubeSources, "cubeSources");
        Objects.requireNonNull(motionType, "motionType");
        if (bodyBone.name == null || bodyBone.name.isBlank()) {
            throw new IllegalArgumentException("bone name is required");
        }
        if (!Double.isFinite(unitScale) || !(unitScale > 0)) {
            throw new IllegalArgumentException("unit scale must be finite and greater than zero");
        }

        List<RigidBodyBackend.BoxShape> boxes = new ArrayList<>();
        int skipped = 0;
        int sourceCubeCount = 0;
        double smallestHalfExtent = Double.POSITIVE_INFINITY;
        double[] inverseBodyRotation = RotationUtil.quaternionInverse(
                bodyPose.worldRotation);

        for (CubeSource source : cubeSources) {
            Objects.requireNonNull(source, "cube source");
            BedrockModelData.Bone sourceBone = source.bone();
            List<BedrockModelData.Cube> cubes = sourceBone.cubes == null
                    ? List.of() : sourceBone.cubes;
            sourceCubeCount += cubes.size();
            for (BedrockModelData.Cube cube : cubes) {
                double[] effectiveSize = CubeGeometry.effectiveSize(cube);
                double[] halfExtents = new double[]{
                        effectiveSize[0] * unitScale * 0.5,
                        effectiveSize[1] * unitScale * 0.5,
                        effectiveSize[2] * unitScale * 0.5
                };
                if (halfExtents[0] <= MIN_HALF_EXTENT
                        || halfExtents[1] <= MIN_HALF_EXTENT
                        || halfExtents[2] <= MIN_HALF_EXTENT) {
                    skipped++;
                    continue;
                }

                double[] bindVertices = CubeGeometry.bindVertices(cube);
                double[] bindCenter = new double[3];
                for (int vertex = 0; vertex < 8; vertex++) {
                    int offset = vertex * 3;
                    bindCenter[0] += bindVertices[offset] / 8.0;
                    bindCenter[1] += bindVertices[offset + 1] / 8.0;
                    bindCenter[2] += bindVertices[offset + 2] / 8.0;
                }
                double[] worldCenter = new double[3];
                CubeGeometry.transformPoint(source.pose(), bindCenter[0],
                        bindCenter[1], bindCenter[2], worldCenter);
                double[] centerDelta = new double[]{
                        worldCenter[0] - bodyPose.worldPosition[0],
                        worldCenter[1] - bodyPose.worldPosition[1],
                        worldCenter[2] - bodyPose.worldPosition[2]
                };
                double[] center = RotationUtil.rotateVector(
                        inverseBodyRotation, centerDelta);
                for (int axis = 0; axis < 3; axis++) center[axis] *= unitScale;

                double[] cubeRotation = cube.rotation == null
                        ? new double[]{0, 0, 0, 1}
                        : RotationUtil.quaternionFromBedrockEuler(
                        cube.rotation[0], cube.rotation[1], cube.rotation[2]);
                double[] sourceWorldRotation = RotationUtil.quaternionMultiply(
                        source.pose().worldRotation, cubeRotation);
                double[] localRotation = RotationUtil.quaternionMultiply(
                        inverseBodyRotation, sourceWorldRotation);
                boxes.add(new RigidBodyBackend.BoxShape(halfExtents,
                        new RigidBodyBackend.Transform(center, localRotation)));
                for (double halfExtent : halfExtents) {
                    smallestHalfExtent = Math.min(smallestHalfExtent, halfExtent);
                }
            }
        }

        if (boxes.isEmpty()) {
            return new Compilation(Optional.empty(), sourceCubeCount, skipped,
                    "bone has no non-degenerate cube in its owned groups: "
                            + bodyBone.name);
        }

        double effectiveMass = motionType == RigidBodyBackend.MotionType.DYNAMIC
                ? mass : 0;
        RigidBodyBackend.CcdSettings ccd =
                motionType == RigidBodyBackend.MotionType.DYNAMIC && enableCcd
                        ? new RigidBodyBackend.CcdSettings(true,
                        smallestHalfExtent * 0.5, smallestHalfExtent * 0.8)
                        : RigidBodyBackend.CcdSettings.disabled();
        RigidBodyBackend.Transform initialTransform = new RigidBodyBackend.Transform(
                new double[]{bodyPose.worldPosition[0] * unitScale,
                        bodyPose.worldPosition[1] * unitScale,
                        bodyPose.worldPosition[2] * unitScale},
                bodyPose.worldRotation);
        RigidBodyBackend.BodyDefinition definition =
                new RigidBodyBackend.BodyDefinition(bodyBone.name, motionType,
                        boxes, initialTransform, effectiveMass,
                        friction, restitution, ccd);
        String diagnostic = skipped == 0 ? ""
                : "skipped " + skipped + " degenerate cube(s) owned by "
                + bodyBone.name;
        return new Compilation(Optional.of(definition), sourceCubeCount, skipped,
                diagnostic);
    }
}
