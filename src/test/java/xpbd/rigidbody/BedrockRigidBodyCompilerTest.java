package xpbd.rigidbody;

import org.junit.jupiter.api.Test;
import xpbd.baker.BonePoseCalculator;
import xpbd.baker.CubeGeometry;
import xpbd.baker.RotationUtil;
import xpbd.loader.BedrockModelData;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

final class BedrockRigidBodyCompilerTest {
    @Test
    void descendantCubeKeepsWorldTransformInOwnerCompoundFrame() {
        BedrockModelData.Bone owner = new BedrockModelData.Bone();
        owner.name = "owner";
        owner.pivot = new double[]{1, 2, 0};
        owner.rotation = new double[]{0, 0, 30};

        BedrockModelData.Bone child = new BedrockModelData.Bone();
        child.name = "child_group";
        child.parent = owner.name;
        child.pivot = new double[]{4, 2, 0};
        child.rotation = new double[]{0, 0, 45};
        BedrockModelData.Cube cube = new BedrockModelData.Cube();
        cube.origin = new double[]{3, 1, -1};
        cube.size = new double[]{2, 2, 2};
        cube.pivot = child.pivot.clone();
        cube.rotation = new double[]{0, 0, 15};
        child.cubes.add(cube);

        var poses = BonePoseCalculator.calculate(List.of(owner, child), null, 0);
        double unitScale = 0.25;
        BedrockRigidBodyCompiler.Compilation result =
                BedrockRigidBodyCompiler.compileCompound(
                        owner, poses.get(owner.name),
                        List.of(new BedrockRigidBodyCompiler.CubeSource(
                                child, poses.get(child.name))),
                        RigidBodyBackend.MotionType.DYNAMIC, 1, unitScale,
                        0.5, 0, true);

        RigidBodyBackend.BodyDefinition body = result.body().orElseThrow();
        RigidBodyBackend.BoxShape box = body.boxes().get(0);
        double[] reconstructedCenter = RotationUtil.rotateVector(
                body.initialBoneTransform().rotation(),
                box.localTransform().translation());
        double[] bodyTranslation = body.initialBoneTransform().translation();
        for (int axis = 0; axis < 3; axis++) {
            reconstructedCenter[axis] += bodyTranslation[axis];
        }

        double[] vertices = CubeGeometry.bindVertices(cube);
        double[] bindCenter = new double[3];
        for (int vertex = 0; vertex < 8; vertex++) {
            for (int axis = 0; axis < 3; axis++) {
                bindCenter[axis] += vertices[vertex * 3 + axis] / 8.0;
            }
        }
        double[] expectedCenter = new double[3];
        CubeGeometry.transformPoint(poses.get(child.name), bindCenter[0],
                bindCenter[1], bindCenter[2], expectedCenter);
        for (int axis = 0; axis < 3; axis++) expectedCenter[axis] *= unitScale;
        assertArrayEquals(expectedCenter, reconstructedCenter, 1e-12);

        double[] reconstructedRotation = RotationUtil.quaternionMultiply(
                body.initialBoneTransform().rotation(),
                box.localTransform().rotation());
        double[] expectedRotation = RotationUtil.quaternionMultiply(
                poses.get(child.name).worldRotation,
                RotationUtil.quaternionFromBedrockEuler(0, 0, 15));
        assertArrayEquals(RotationUtil.rotateVector(expectedRotation,
                        new double[]{1, 0, 0}),
                RotationUtil.rotateVector(reconstructedRotation,
                        new double[]{1, 0, 0}), 1e-12);
    }

    @Test
    void rotatedInflatedCubeCompilesInBonePivotFrame() {
        BedrockModelData.Bone bone = new BedrockModelData.Bone();
        bone.name = "cloth";
        bone.pivot = new double[]{1, 2, 3};
        BedrockModelData.Cube cube = new BedrockModelData.Cube();
        cube.origin = new double[]{1, 2, 3};
        cube.size = new double[]{2, 4, 6};
        cube.pivot = new double[]{2, 4, 6};
        cube.rotation = new double[]{0, 0, 90};
        cube.inflate = 0.5;
        bone.cubes.add(cube);

        BonePoseCalculator.Pose pose = BonePoseCalculator
                .calculate(List.of(bone), null, 0).get(bone.name);
        BedrockRigidBodyCompiler.Compilation result =
                BedrockRigidBodyCompiler.compile(bone, pose,
                        RigidBodyBackend.MotionType.DYNAMIC, 2, 0.25);

        assertTrue(result.body().isPresent());
        assertEquals(1, result.sourceCubeCount());
        assertEquals(0, result.skippedDegenerateCubeCount());
        RigidBodyBackend.BodyDefinition body = result.body().orElseThrow();
        RigidBodyBackend.BoxShape box = body.boxes().get(0);
        assertArrayEquals(new double[]{0.375, 0.625, 0.875},
                box.halfExtents(), 1e-12);

        double[] vertices = CubeGeometry.bindVertices(cube);
        double[] expectedCenter = new double[3];
        for (int vertex = 0; vertex < 8; vertex++) {
            for (int axis = 0; axis < 3; axis++) {
                expectedCenter[axis] += vertices[vertex * 3 + axis] / 8.0;
            }
        }
        double[] mappedPivot = new double[]{
                -bone.pivot[0], bone.pivot[1], bone.pivot[2]};
        for (int axis = 0; axis < 3; axis++) {
            expectedCenter[axis] = (expectedCenter[axis] - mappedPivot[axis]) * 0.25;
        }
        assertArrayEquals(expectedCenter,
                box.localTransform().translation(), 1e-12);
        assertTrue(body.ccd().enabled());
        assertEquals(2, body.mass(), 0);
    }

    @Test
    void boneWithoutSolidCubeIsDiagnosedAndSkipped() {
        BedrockModelData.Bone bone = new BedrockModelData.Bone();
        bone.name = "marker";
        BedrockModelData.Cube cube = new BedrockModelData.Cube();
        cube.origin = new double[]{0, 0, 0};
        cube.size = new double[]{0, 2, 2};
        bone.cubes.add(cube);
        BonePoseCalculator.Pose pose = BonePoseCalculator
                .calculate(List.of(bone), null, 0).get(bone.name);

        BedrockRigidBodyCompiler.Compilation result =
                BedrockRigidBodyCompiler.compile(bone, pose,
                        RigidBodyBackend.MotionType.KINEMATIC, 0, 1);

        assertFalse(result.body().isPresent());
        assertEquals(1, result.skippedDegenerateCubeCount());
        assertTrue(result.diagnostic().contains("no non-degenerate cube"));
    }
}
