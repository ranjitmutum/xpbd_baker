package xpbd.baker;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import javafx.geometry.Point3D;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.shape.Box;
import javafx.scene.transform.Affine;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import org.junit.jupiter.api.Test;
import xpbd.loader.BedrockModelData;
import xpbd.loader.ModelLoader;
import xpbd.rigidbody.BedrockRigidBodyCompiler;
import xpbd.rigidbody.RigidBodyBackend;

import java.io.Reader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

final class BedrockTransformResolverTest {
    private static final double EPSILON = 1e-9;

    @Test
    void parserPreservesTransformNumbersAndParentExactly() throws Exception {
        Path path = fixturePath();
        JsonObject root;
        try (Reader reader = Files.newBufferedReader(path, StandardCharsets.UTF_8)) {
            root = JsonParser.parseReader(reader).getAsJsonObject();
        }
        BedrockModelData.Geometry parsed = ModelLoader.load(path.toString());
        JsonArray rawBones = root.getAsJsonArray("minecraft:geometry")
                .get(0).getAsJsonObject().getAsJsonArray("bones");

        assertEquals(rawBones.size(), parsed.bones.size());
        for (int boneIndex = 0; boneIndex < rawBones.size(); boneIndex++) {
            JsonObject rawBone = rawBones.get(boneIndex).getAsJsonObject();
            BedrockModelData.Bone bone = parsed.bones.get(boneIndex);
            assertEquals(rawBone.get("name").getAsString(), bone.name);
            assertEquals(rawBone.has("parent")
                    ? rawBone.get("parent").getAsString() : null, bone.parent);
            assertRawVector(rawBone.getAsJsonArray("pivot"), bone.pivot);
            assertRawVector(rawBone.getAsJsonArray("rotation"), bone.rotation);
            JsonArray rawCubes = rawBone.getAsJsonArray("cubes");
            assertEquals(rawCubes.size(), bone.cubes.size());
            for (int cubeIndex = 0; cubeIndex < rawCubes.size(); cubeIndex++) {
                JsonObject rawCube = rawCubes.get(cubeIndex).getAsJsonObject();
                BedrockModelData.Cube cube = bone.cubes.get(cubeIndex);
                assertRawVector(rawCube.getAsJsonArray("origin"), cube.origin);
                assertRawVector(rawCube.getAsJsonArray("size"), cube.size);
                if (rawCube.has("pivot")) {
                    assertRawVector(rawCube.getAsJsonArray("pivot"), cube.pivot);
                }
                assertRawVector(rawCube.getAsJsonArray("rotation"), cube.rotation);
                assertEquals(rawCube.get("inflate").getAsDouble(), cube.inflate, 0);
            }
        }
    }

    @Test
    void singleAxisRotationsMatchBlockbenchBedrockImportMapping() {
        assertRotated(new double[]{90, 0, 0}, new double[]{0, 1, 0},
                new double[]{0, 0, -1});
        assertRotated(new double[]{0, 90, 0}, new double[]{0, 0, 1},
                new double[]{-1, 0, 0});
        assertRotated(new double[]{0, 0, 90}, new double[]{1, 0, 0},
                new double[]{0, 1, 0});
    }

    @Test
    void vectorsMirrorXExactlyLikeBlockbenchImporter() {
        assertArrayEquals(new double[]{-3.25, -4.5, 5.75},
                BedrockTransformResolver.convertBedrockVector(
                        new double[]{3.25, -4.5, 5.75}), EPSILON);
    }

    @Test
    void multiAxisAndNearHalfTurnMatricesMatchQuaternionPath() {
        List<double[]> rotations = List.of(
                new double[]{12.5, -37.25, 81.75},
                new double[]{89.999, -90.001, 179.999},
                new double[]{-90, 180, -180});
        double[] source = {2.25, -3.5, 4.75};
        for (double[] rotation : rotations) {
            BedrockModelData.Bone bone = bone("test", null,
                    new double[3], rotation);
            double[] mappedSource = BedrockTransformResolver
                    .convertBedrockVector(source);
            double[] matrixResult = BedrockTransformResolver
                    .resolveBoneLocalMatrix(bone)
                    .transformPoint(mappedSource[0], mappedSource[1], mappedSource[2]);
            double[] quaternionResult = RotationUtil.rotateVector(
                    RotationUtil.quaternionFromBedrockEuler(
                            rotation[0], rotation[1], rotation[2]), mappedSource);
            assertArrayEquals(quaternionResult, matrixResult, EPSILON);
        }
    }

    @Test
    void resolverMatchesBlockbenchImportedJavaFxTransformOrder() {
        double[] pivot = {3.25, -4.5, 5.75};
        double[] rotation = {17.5, -28.25, 63.75};
        double[] source = {-6.5, 7.25, 8.75};
        double[] mappedPivot = BedrockTransformResolver.convertBedrockVector(pivot);
        double[] mappedSource = BedrockTransformResolver.convertBedrockVector(source);
        BedrockModelData.Bone bone = bone("legacy", null, pivot, rotation);
        Group former = new Group();
        former.getTransforms().addAll(
                new Translate(mappedPivot[0], mappedPivot[1], mappedPivot[2]),
                new Rotate(rotation[2], Rotate.Z_AXIS),
                new Rotate(-rotation[1], Rotate.Y_AXIS),
                new Rotate(-rotation[0], Rotate.X_AXIS),
                new Translate(-mappedPivot[0], -mappedPivot[1], -mappedPivot[2]));
        Point3D formerPoint = former.localToParent(
                mappedSource[0], mappedSource[1], mappedSource[2]);
        double[] resolved = BedrockTransformResolver.resolveBoneLocalMatrix(bone)
                .transformPoint(mappedSource[0], mappedSource[1], mappedSource[2]);

        assertArrayEquals(resolved, new double[]{formerPoint.getX(),
                formerPoint.getY(), formerPoint.getZ()}, EPSILON);
    }

    @Test
    void parentTransformIsAppliedExactlyOnce() {
        BedrockModelData.Bone parent = bone("parent", null,
                new double[]{1, 2, 3}, new double[]{15, -25, 35});
        BedrockModelData.Bone child = bone("child", "parent",
                new double[]{7, -5, 4}, new double[]{-45, 55, -65});
        List<BedrockModelData.Bone> bones = List.of(parent, child);
        Map<String, BedrockTransformResolver.Matrix4> matrices =
                BedrockTransformResolver.resolveBoneWorldMatrices(bones);
        Map<String, BonePoseCalculator.Pose> poses =
                BonePoseCalculator.calculate(bones, null, 0);
        double[] source = {8.5, -2.25, 6.75};
        double[] expected = new double[3];
        CubeGeometry.transformPoint(poses.get("child"),
                source[0], source[1], source[2], expected);

        assertArrayEquals(expected, matrices.get("child").transformPoint(
                source[0], source[1], source[2]), EPSILON);
        assertArrayEquals(poses.get("child").worldPosition,
                matrices.get("child").transformPoint(
                        -child.pivot[0], child.pivot[1], child.pivot[2]), EPSILON);
    }

    @Test
    void resolverMatchesBlockbenchParentOriginSubtractionForCubeCenter() {
        BedrockModelData.Bone parent = bone("parent", null,
                new double[]{1, 2, 3}, new double[]{15, -25, 35});
        BedrockModelData.Bone child = bone("child", "parent",
                new double[]{7, -5, 4}, new double[]{-45, 55, -65});
        BedrockModelData.Cube cube = new BedrockModelData.Cube();
        cube.origin = new double[]{5, -7, 2};
        cube.size = new double[]{4, 6, 8};
        cube.pivot = new double[]{8, -3, 5};
        cube.rotation = new double[]{20, -30, 40};
        child.cubes.add(cube);

        Group parentGroup = new Group();
        Group childGroup = new Group();
        Box box = new Box(4, 6, 8);
        parentGroup.getChildren().add(childGroup);
        childGroup.getChildren().add(box);
        double[] parentOrigin = {-parent.pivot[0], parent.pivot[1], parent.pivot[2]};
        double[] childOrigin = {-child.pivot[0], child.pivot[1], child.pivot[2]};
        double[] cubePivot = {-cube.pivot[0], cube.pivot[1], cube.pivot[2]};
        double[] cubeCenter = {-(cube.origin[0] + cube.size[0] * 0.5),
                cube.origin[1] + cube.size[1] * 0.5,
                cube.origin[2] + cube.size[2] * 0.5};
        parentGroup.getTransforms().add(new Translate(
                parentOrigin[0], parentOrigin[1], parentOrigin[2]));
        addBlockbenchRotations(parentGroup, parent.rotation);
        childGroup.getTransforms().add(new Translate(
                childOrigin[0] - parentOrigin[0],
                childOrigin[1] - parentOrigin[1],
                childOrigin[2] - parentOrigin[2]));
        addBlockbenchRotations(childGroup, child.rotation);
        box.getTransforms().add(new Translate(
                cubePivot[0] - childOrigin[0],
                cubePivot[1] - childOrigin[1],
                cubePivot[2] - childOrigin[2]));
        addBlockbenchRotations(box, cube.rotation);
        box.getTransforms().add(new Translate(
                cubeCenter[0] - cubePivot[0],
                cubeCenter[1] - cubePivot[1],
                cubeCenter[2] - cubePivot[2]));

        Point3D expected = box.localToScene(0, 0, 0);
        Map<String, BedrockTransformResolver.Matrix4> worlds =
                BedrockTransformResolver.resolveBoneWorldMatrices(
                        List.of(parent, child));
        double[] actual = transformCubePoint(
                worlds.get(child.name), cube, 0, 0, 0);
        assertArrayEquals(new double[]{expected.getX(), expected.getY(), expected.getZ()},
                actual, EPSILON);
    }

    @Test
    void resolverGeometryAndJavaFxLocalToSceneAgreeForEveryFixtureCorner()
            throws Exception {
        BedrockModelData.Geometry geometry = ModelLoader.load(fixturePath().toString());
        Map<String, BedrockTransformResolver.Matrix4> worlds =
                BedrockTransformResolver.resolveBoneWorldMatrices(geometry.bones);
        Map<String, BonePoseCalculator.Pose> poses =
                BonePoseCalculator.calculate(geometry.bones, null, 0);

        Group root = new Group();
        Map<String, Group> boneGroups = new java.util.LinkedHashMap<>();
        for (BedrockModelData.Bone bone : geometry.bones) {
            Group group = new Group();
            group.getTransforms().add(toAffine(
                    BedrockTransformResolver.resolveBoneLocalMatrix(bone)));
            boneGroups.put(bone.name, group);
            if (bone.parent == null) root.getChildren().add(group);
            else boneGroups.get(bone.parent).getChildren().add(group);

            for (BedrockModelData.Cube cube : bone.cubes) {
                double[] size = CubeGeometry.effectiveSize(cube);
                Box box = new Box(size[0], size[1], size[2]);
                box.getTransforms().add(toAffine(
                        BedrockTransformResolver.resolveCubeLocalMatrix(cube)));
                group.getChildren().add(box);

                double[] bind = CubeGeometry.bindVertices(cube);
                for (int z = 0; z < 2; z++) {
                    for (int y = 0; y < 2; y++) {
                        for (int x = 0; x < 2; x++) {
                            int vertex = x + y * 2 + z * 4;
                            double lx = (x - 0.5) * size[0];
                            double ly = (y - 0.5) * size[1];
                            double lz = (z - 0.5) * size[2];
                            double[] resolverPoint = transformCubePoint(
                                    worlds.get(bone.name), cube, lx, ly, lz);
                            double[] geometryPoint = new double[3];
                            CubeGeometry.transformPoint(poses.get(bone.name),
                                    bind[vertex * 3], bind[vertex * 3 + 1],
                                    bind[vertex * 3 + 2], geometryPoint);
                            Point3D javaFxPoint = box.localToScene(lx, ly, lz);
                            assertArrayEquals(geometryPoint, resolverPoint, EPSILON);
                            assertArrayEquals(resolverPoint, new double[]{
                                    javaFxPoint.getX(), javaFxPoint.getY(),
                                    javaFxPoint.getZ()}, EPSILON);
                        }
                    }
                }
            }
        }
    }

    @Test
    void negativeSizesCanonicalizeBoundsAndKeepTheSameCenter() throws Exception {
        BedrockModelData.Geometry geometry = ModelLoader.load(fixturePath().toString());
        BedrockModelData.Cube cube = geometry.bones.get(1).cubes.get(0);

        assertArrayEquals(new double[]{4.75, -8.5, 5.0},
                CubeGeometry.effectiveOrigin(cube), EPSILON);
        assertArrayEquals(new double[]{3.0, 4.0, 5.0},
                CubeGeometry.effectiveSize(cube), EPSILON);
        assertArrayEquals(new double[]{6.25, -6.5, 7.5},
                CubeGeometry.effectivePivot(cube), EPSILON);
        double[] vertices = CubeGeometry.bindVertices(cube);
        for (double value : vertices) assertTrue(Double.isFinite(value));
    }

    @Test
    void negativeSizeStillRejectsInflateThatInvertsEffectiveMagnitude() {
        assertThrows(IllegalArgumentException.class, () ->
                BedrockModelData.Cube.fromJson(JsonParser.parseString("""
                        {"origin":[0,0,0],"size":[-1,2,3],"inflate":-0.6}
                        """).getAsJsonObject()));
    }

    @Test
    void negativeSizeProducesPositiveRigidBodyHalfExtents() throws Exception {
        BedrockModelData.Bone bone = ModelLoader.load(fixturePath().toString())
                .bones.get(1);
        BonePoseCalculator.Pose pose = BonePoseCalculator
                .calculate(List.of(bone), null, 0).get(bone.name);
        BedrockRigidBodyCompiler.Compilation compiled =
                BedrockRigidBodyCompiler.compile(bone, pose,
                        RigidBodyBackend.MotionType.STATIC, 0, 0.5);

        RigidBodyBackend.BoxShape box = compiled.body().orElseThrow()
                .boxes().get(0);
        assertArrayEquals(new double[]{0.75, 1.0, 1.25},
                box.halfExtents(), EPSILON);
    }

    @Test
    void suppliedComplexModelMatchesExistingBindPosePath() throws Exception {
        String modelPath = System.getProperty("xpbd.test.shiroModel");
        assumeTrue(modelPath != null && !modelPath.isBlank(),
                "set -Dxpbd.test.shiroModel to run the external asset regression");
        BedrockModelData.Geometry geometry = ModelLoader.load(modelPath);
        Map<String, BedrockTransformResolver.Matrix4> worlds =
                BedrockTransformResolver.resolveBoneWorldMatrices(geometry.bones);
        Map<String, BonePoseCalculator.Pose> poses =
                BonePoseCalculator.calculate(geometry.bones, null, 0);
        int cubeCount = 0;
        for (BedrockModelData.Bone bone : geometry.bones) {
            BedrockTransformResolver.Matrix4 world = worlds.get(bone.name);
            assertNotNull(world);
            double[] mappedPivot = BedrockTransformResolver.convertBedrockVector(bone.pivot);
            assertArrayEquals(poses.get(bone.name).worldPosition,
                    world.transformPoint(mappedPivot[0], mappedPivot[1], mappedPivot[2]),
                    1e-8, bone.name);
            for (BedrockModelData.Cube cube : bone.cubes) {
                cubeCount++;
                double[] bind = CubeGeometry.bindVertices(cube);
                double[] size = CubeGeometry.effectiveSize(cube);
                for (int offset = 0; offset < bind.length; offset += 3) {
                    double[] existing = new double[3];
                    CubeGeometry.transformPoint(poses.get(bone.name),
                            bind[offset], bind[offset + 1], bind[offset + 2], existing);
                    int vertex = offset / 3;
                    int x = vertex & 1;
                    int y = (vertex >> 1) & 1;
                    int z = (vertex >> 2) & 1;
                    double[] resolved = transformCubePoint(world, cube,
                            (x - 0.5) * size[0],
                            (y - 0.5) * size[1],
                            (z - 0.5) * size[2]);
                    assertTrue(Double.isFinite(existing[0])
                            && Double.isFinite(existing[1])
                            && Double.isFinite(existing[2]));
                    assertArrayEquals(existing, resolved, 1e-8,
                            bone.name + " cube " + (cubeCount - 1)
                                    + " vertex " + vertex);
                }
            }
        }
        assertTrue(cubeCount > 0);
    }

    private static void assertRotated(double[] rotation, double[] source,
                                      double[] expected) {
        BedrockModelData.Bone bone = bone("axis", null, new double[3], rotation);
        assertArrayEquals(expected, BedrockTransformResolver
                .resolveBoneLocalMatrix(bone)
                .transformPoint(source[0], source[1], source[2]), EPSILON);
    }

    private static void addBlockbenchRotations(Node node, double[] rotation) {
        node.getTransforms().addAll(
                new Rotate(rotation[2], Rotate.Z_AXIS),
                new Rotate(-rotation[1], Rotate.Y_AXIS),
                new Rotate(-rotation[0], Rotate.X_AXIS));
    }

    private static BedrockModelData.Bone bone(String name, String parent,
                                               double[] pivot, double[] rotation) {
        BedrockModelData.Bone bone = new BedrockModelData.Bone();
        bone.name = name;
        bone.parent = parent;
        bone.pivot = pivot;
        bone.rotation = rotation;
        return bone;
    }

    private static void assertRawVector(JsonArray raw, double[] parsed) {
        assertNotNull(raw);
        assertNotNull(parsed);
        assertEquals(3, parsed.length);
        for (int axis = 0; axis < 3; axis++) {
            assertEquals(raw.get(axis).getAsDouble(), parsed[axis], 0);
        }
    }

    private static Affine toAffine(BedrockTransformResolver.Matrix4 matrix) {
        return new Affine(
                matrix.get(0, 0), matrix.get(0, 1), matrix.get(0, 2), matrix.get(0, 3),
                matrix.get(1, 0), matrix.get(1, 1), matrix.get(1, 2), matrix.get(1, 3),
                matrix.get(2, 0), matrix.get(2, 1), matrix.get(2, 2), matrix.get(2, 3));
    }

    private static double[] transformCubePoint(
            BedrockTransformResolver.Matrix4 boneWorld,
            BedrockModelData.Cube cube,
            double x, double y, double z) {
        double[] boneLocal = BedrockTransformResolver.resolveCubeLocalMatrix(cube)
                .transformPoint(x, y, z);
        return boneWorld.transformPoint(
                boneLocal[0], boneLocal[1], boneLocal[2]);
    }

    private static Path fixturePath() throws Exception {
        return Path.of(BedrockTransformResolverTest.class.getResource(
                "/bedrock/transform_baselines.geo.json").toURI());
    }
}
