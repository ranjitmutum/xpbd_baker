package xpbd.regression;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Test;
import xpbd.baker.BodyColliderCache;
import xpbd.baker.BoneMapper;
import xpbd.baker.BonePoseCalculator;
import xpbd.baker.CubeGeometry;
import xpbd.baker.PhysicsBaker;
import xpbd.baker.RotationUtil;
import xpbd.loader.BedrockAnimationData;
import xpbd.loader.BedrockModelData;
import xpbd.loader.ModelLoader;
import xpbd.rigidbody.BedrockRigidBodyCompiler;
import xpbd.rigidbody.RigidBodyBackend;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.nio.file.Path;
import java.nio.file.Files;
import java.nio.charset.StandardCharsets;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

final class RealModelCompatibilityTest {
    @Test
    void shiroHurtWalkNumericRootScaleInitializesRigidRibbons() throws Exception {
        String modelPath = System.getProperty("xpbd.test.shiroModel");
        assumeTrue(modelPath != null && !modelPath.isBlank(),
                "set -Dxpbd.test.shiroModel to run the external asset smoke test");

        Path animationPath = Path.of(modelPath).getParent().getParent()
                .resolve("animations").resolve("main.animation.json");
        BedrockModelData.Geometry geometry = ModelLoader.load(modelPath);
        JsonObject animationJson;
        try (var reader = Files.newBufferedReader(animationPath,
                StandardCharsets.UTF_8)) {
            animationJson = JsonParser.parseReader(reader).getAsJsonObject();
        }
        BedrockAnimationData.Animation hurtWalk = BedrockAnimationData.Animation.fromJson(
                animationJson.getAsJsonObject("animations")
                        .getAsJsonObject("hurt_walk"));
        assertTrue(hurtWalk.bones.get("Root").scale.keyframes.values().stream()
                .anyMatch(value -> Math.abs(value[1] - 1) > 1e-9));

        BoneMapper mapper = new BoneMapper(geometry.bones);
        List.of("Right_Line", "Right_Line2", "Right_Line3",
                        "Left_Line", "Left_Line2", "Left_Line3")
                .forEach(mapper::addPhysicsBone);
        BoneMapper.PhysicsGroupConfig config = mapper.getConfig();
        config.simulationMode = BoneMapper.SimulationMode.RIGID_BODY;
        config.gravityY = 0;
        config.windSpeed = 0;
        config.airDrag = 0;
        config.turbulence = 0;
        config.animationPullCompliance = 0;
        config.rigidBodyMaximumSafePenetration = 10;

        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setSourceAnimation(hurtWalk);
            baker.initialize();
            baker.runSteps(2);
            assertEquals(6, baker.getRigidBodyPhysicsBodyCount());
            assertEquals(2, baker.getCurrentStep());
        }
    }

    @Test
    void userSuppliedShiroModelRemainsCompactAfterBedrockRotations() throws Exception {
        String modelPath = System.getProperty("xpbd.test.shiroModel");
        assumeTrue(modelPath != null && !modelPath.isBlank(),
                "set -Dxpbd.test.shiroModel to run the external asset smoke test");

        BedrockModelData.Geometry geometry = ModelLoader.load(modelPath);
        assertEquals(638, geometry.bones.size());
        assertEquals(1047, geometry.bones.stream()
                .mapToInt(bone -> bone.cubes.size()).sum());

        Map<String, BonePoseCalculator.Pose> poses =
                BonePoseCalculator.calculate(geometry.bones, null, 0);
        List<Double> absoluteCenterZ = new ArrayList<>();
        double[] transformed = new double[3];
        for (BedrockModelData.Bone bone : geometry.bones) {
            BonePoseCalculator.Pose pose = poses.get(bone.name);
            for (BedrockModelData.Cube cube : bone.cubes) {
                double[] vertices = CubeGeometry.bindVertices(cube);
                double centerZ = 0;
                for (int vertex = 0; vertex < 8; vertex++) {
                    int offset = vertex * 3;
                    CubeGeometry.transformPoint(pose, vertices[offset],
                            vertices[offset + 1], vertices[offset + 2], transformed);
                    centerZ += transformed[2];
                }
                absoluteCenterZ.add(Math.abs(centerZ / 8.0));
            }
        }

        Collections.sort(absoluteCenterZ);
        double p99 = absoluteCenterZ.get((int) Math.floor(
                0.99 * (absoluteCenterZ.size() - 1)));
        assertTrue(p99 < 25,
                "model has scattered cubes; expected p99 |centerZ| < 25, got " + p99);

        Set<String> allBones = geometry.bones.stream()
                .map(bone -> bone.name).collect(Collectors.toSet());
        BodyColliderCache cache = new BodyColliderCache(geometry.bones, allBones);
        cache.initialize(poses);
        assertEquals(1041, cache.getColliders().size());
        assertEquals(6, cache.getDegenerateCubeCount());

        int compiledBodies = 0;
        int compiledBoxes = 0;
        int skippedDegenerateBoxes = 0;
        Set<String> usableRigidBodyBones = new HashSet<>();
        for (BedrockModelData.Bone bone : geometry.bones) {
            BedrockRigidBodyCompiler.Compilation compiled =
                    BedrockRigidBodyCompiler.compile(bone, poses.get(bone.name),
                            RigidBodyBackend.MotionType.STATIC, 0, 1);
            if (compiled.body().isPresent()) {
                compiledBodies++;
                compiledBoxes += compiled.body().orElseThrow().boxes().size();
                usableRigidBodyBones.add(bone.name);
            }
            skippedDegenerateBoxes += compiled.skippedDegenerateCubeCount();
        }
        assertTrue(compiledBodies > 100);
        assertEquals(1041, compiledBoxes);
        assertEquals(6, skippedDegenerateBoxes);

        BoneMapper mapper = new BoneMapper(geometry.bones);
        mapper.addCollisionRoot("RightLeg");
        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.initialize();
            assertTrue(baker.getBodyColliderCount() > 0);
        }

        BedrockModelData.Bone rigidChild = geometry.bones.stream()
                .filter(bone -> bone.parent != null)
                .filter(bone -> usableRigidBodyBones.contains(bone.name))
                .filter(bone -> usableRigidBodyBones.contains(bone.parent))
                .findFirst().orElseThrow();
        BoneMapper rigidMapper = new BoneMapper(geometry.bones);
        rigidMapper.addPhysicsBone(rigidChild.parent);
        rigidMapper.addPhysicsBone(rigidChild.name);
        rigidMapper.getConfig().simulationMode = BoneMapper.SimulationMode.RIGID_BODY;
        rigidMapper.getConfig().gravityY = 0;
        rigidMapper.getConfig().windSpeed = 0;
        rigidMapper.getConfig().airDrag = 0;
        rigidMapper.getConfig().turbulence = 0;
        rigidMapper.getConfig().animationPullCompliance = 0;
        rigidMapper.getConfig().rigidBodyMaximumSafePenetration = 10;
        try (PhysicsBaker rigidBaker = new PhysicsBaker(rigidMapper)) {
            rigidBaker.initialize();
            rigidBaker.runSteps(2);
            assertTrue(rigidBaker.isRigidBodyMode());
            assertEquals(2, rigidBaker.getRigidBodyPhysicsBodyCount());
            assertTrue(rigidBaker.getFrames().size() >= 3);
        }
    }

    @Test
    void shiroRunRibbonRootsFollowAnimationAcrossSweepHits() throws Exception {
        String modelPath = System.getProperty("xpbd.test.shiroModel");
        assumeTrue(modelPath != null && !modelPath.isBlank(),
                "set -Dxpbd.test.shiroModel to run the external asset smoke test");

        Path animationPath = Path.of(modelPath).getParent().getParent()
                .resolve("animations").resolve("main.animation.json");
        BedrockModelData.Geometry geometry = ModelLoader.load(modelPath);
        JsonObject animationJson;
        try (var reader = Files.newBufferedReader(animationPath,
                StandardCharsets.UTF_8)) {
            animationJson = JsonParser.parseReader(reader).getAsJsonObject();
        }
        JsonObject runJson = animationJson.getAsJsonObject("animations")
                .getAsJsonObject("run").deepCopy();
        runJson.getAsJsonObject("bones").remove("molang");
        BedrockAnimationData.Animation run =
                BedrockAnimationData.Animation.fromJson(runJson);

        BoneMapper mapper = new BoneMapper(geometry.bones);
        List<String> ribbonBones = List.of(
                "Right_Line", "Right_Line2",
                "Left_Line", "Left_Line2");
        ribbonBones.forEach(mapper::addPhysicsBone);
        mapper.addCollisionRoot("RightLeg");
        mapper.addCollisionRoot("LeftLeg");
        BoneMapper.PhysicsGroupConfig config = mapper.getConfig();
        config.simulationMode = BoneMapper.SimulationMode.RIGID_BODY;
        config.rigidBodySubsteps = 1;
        config.movementSpeed = 4.2;
        config.movementDirectionDegrees = -90;
        config.movementElevationDegrees = 0;
        config.airDrag = 2;
        config.turbulence = 1.5;
        config.solverIterations = 8;
        config.animationPullCompliance = 0.1;
        config.collisionSkin = 0.1;
        config.rigidBodyMaximumSafePenetration = 10;

        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setSourceAnimation(run);
            baker.initialize();
            assertEquals(1.0 / 60.0, baker.getOutputFrameInterval(), 1e-12);
            assertEquals(4, baker.getRigidBodyPhysicsBodyCount());
            assertEquals(26, baker.getBodyColliderCount());

            int firstSweepStep = -1;
            long previousSweepCount = 0;
            for (int step = 1; step <= 80; step++) {
                baker.runSteps(1);
                assertArrayEquals(baker.getCurrentReferenceWorldPosition("Right_Line"),
                        baker.getCurrentWorldPosition("Right_Line"), 2e-4,
                        "Right_Line kinematic root diverged at step " + step);
                assertArrayEquals(baker.getCurrentReferenceWorldPosition("Left_Line"),
                        baker.getCurrentWorldPosition("Left_Line"), 2e-4,
                        "Left_Line kinematic root diverged at step " + step);
                long sweepCount = baker.getRigidBodySweepHitCount();
                if (firstSweepStep < 0 && sweepCount > previousSweepCount) {
                    firstSweepStep = step;
                }
                previousSweepCount = sweepCount;
            }

            assertTrue(firstSweepStep > 0,
                    "the reference run must exercise at least one kinematic sweep hit");
            System.out.println("Shiro run first kinematic sweep: frame/substep "
                    + firstSweepStep + "/1");
        }
    }

    @Test
    void shiroLuodi2RigidRibbonsReturnToTheSourceEndPose() throws Exception {
        String modelPath = System.getProperty("xpbd.test.shiroModel");
        assumeTrue(modelPath != null && !modelPath.isBlank(),
                "set -Dxpbd.test.shiroModel to run the external asset smoke test");

        Path animationPath = Path.of(modelPath).getParent().getParent()
                .resolve("animations").resolve("main.animation.json");
        BedrockModelData.Geometry geometry = ModelLoader.load(modelPath);
        JsonObject animationJson;
        try (var reader = Files.newBufferedReader(animationPath,
                StandardCharsets.UTF_8)) {
            animationJson = JsonParser.parseReader(reader).getAsJsonObject();
        }
        JsonObject luodi2Json = animationJson.getAsJsonObject("animations")
                .getAsJsonObject("luodi2").deepCopy();
        luodi2Json.getAsJsonObject("bones").remove("molang");
        BedrockAnimationData.Animation luodi2 =
                BedrockAnimationData.Animation.fromJson(luodi2Json);

        BoneMapper mapper = new BoneMapper(geometry.bones);
        List<String> ribbonBones = List.of(
                "Right_Line", "Right_Line2", "Right_Line3",
                "Left_Line", "Left_Line2", "Left_Line3");
        ribbonBones.forEach(mapper::addPhysicsBone);
        mapper.addCollisionRoot("RightLeg");
        mapper.addCollisionRoot("LeftLeg");
        BoneMapper.PhysicsGroupConfig config = mapper.getConfig();
        config.simulationMode = BoneMapper.SimulationMode.RIGID_BODY;
        config.windSpeed = 6;
        config.windDirectionDegrees = 20;
        config.windElevationDegrees = 20;
        config.airDrag = 2;
        config.turbulence = 1.5;
        config.solverIterations = 8;
        config.animationPullCompliance = 0.1;
        config.collisionSkin = 0.1;
        config.transitionDuration = 0.25;
        config.rigidBodyMaximumSafePenetration = 10;

        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setSourceAnimation(luodi2);
            baker.setTransitionAnimation(luodi2);
            baker.initialize();
            baker.runToEnd();

            PhysicsBaker.BakedFrame last = baker.getFrames().get(
                    baker.getFrames().size() - 1);
            Map<String, BonePoseCalculator.Pose> reference =
                    BonePoseCalculator.calculate(
                            geometry.bones, luodi2, luodi2.animationLength);
            for (String boneName : ribbonBones) {
                BedrockModelData.Bone bone = geometry.bones.stream()
                        .filter(candidate -> boneName.equals(candidate.name))
                        .findFirst().orElseThrow();
                PhysicsBaker.BoneState state = last.getBoneState(boneName);
                BonePoseCalculator.Pose target = reference.get(boneName);
                double[] actualQ = RotationUtil.quaternionFromBedrockEuler(
                        bone.rotation[0] + state.rotation[0],
                        bone.rotation[1] + state.rotation[1],
                        bone.rotation[2] + state.rotation[2]);
                double[] targetQ = RotationUtil.quaternionFromBedrockEuler(
                        target.totalLocalEuler[0], target.totalLocalEuler[1],
                        target.totalLocalEuler[2]);

                assertEquals(0, quaternionAngle(actualQ, targetQ), 1e-7,
                        boneName + " did not return to the luodi2 end rotation");
                assertArrayEquals(target.worldPosition, state.worldPosition, 1e-8,
                        boneName + " did not return to the luodi2 end position");
                assertArrayEquals(new double[3], state.linearVelocity, 1e-8,
                        boneName + " retained endpoint velocity");
            }
        }
    }

    private static double quaternionAngle(double[] left, double[] right) {
        double dot = 0;
        for (int i = 0; i < 4; i++) dot += left[i] * right[i];
        return 2 * Math.acos(Math.max(-1, Math.min(1, Math.abs(dot))));
    }
}
