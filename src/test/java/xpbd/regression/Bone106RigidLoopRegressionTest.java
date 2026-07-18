package xpbd.regression;

import org.junit.jupiter.api.Test;
import xpbd.baker.BoneMapper;
import xpbd.baker.LoopSeamReport;
import xpbd.baker.PhysicsBaker;
import xpbd.loader.AnimationLoader;
import xpbd.loader.BedrockAnimationData;
import xpbd.loader.BedrockModelData;
import xpbd.loader.ModelLoader;
import xpbd.export.AnimationExporter;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

final class Bone106RigidLoopRegressionTest {
    private static final List<String> RIGID_BONES = List.of(
            "bone106", "bone107", "bone108", "bone109");

    @Test
    void exportedWalkLoopIsContinuousQuantizedAndCollisionSafe() throws Exception {
        Path modelPath = propertyPath("xpbd.test.loopModel");
        Path animationPath = propertyPath("xpbd.test.loopAnimation");
        assumeTrue(Files.isRegularFile(modelPath) && Files.isRegularFile(animationPath),
                "set xpbd.test.loopModel and xpbd.test.loopAnimation to run this regression");

        BedrockModelData.Geometry geometry = ModelLoader.load(modelPath.toString());
        BedrockAnimationData.Animation animation = AnimationLoader.load(
                animationPath.toString()).animations.get("walk");
        assertNotNull(animation, "walk animation is required");
        BakeResult first = bakeAndValidate(mapper(geometry), animation);
        Path firstExport = Files.createTempFile("bone106-loop-first", ".json");
        Path secondExport = Files.createTempFile("bone106-loop-second", ".json");
        try {
            AnimationExporter.export("animation.loop_regression", animation,
                    first.frames, true, firstExport.toString());
            BedrockAnimationData.Animation reloaded = AnimationLoader.load(
                    firstExport.toString()).animations.get("animation.loop_regression");
            assertNotNull(reloaded);
            for (String boneName : RIGID_BONES) {
                BedrockAnimationData.BoneAnimation channel = reloaded.bones.get(boneName);
                assertArrayEquals(firstValue(channel.position.keyframes),
                        lastValue(channel.position.keyframes), 0,
                        boneName + " reloaded position");
                assertArrayEquals(firstValue(channel.rotation.keyframes),
                        lastValue(channel.rotation.keyframes), 0,
                        boneName + " reloaded rotation");
            }

            BakeResult second = bakeAndValidate(mapper(geometry), animation);
            AnimationExporter.export("animation.loop_regression", animation,
                    second.frames, true, secondExport.toString());
            assertArrayEquals(Files.readAllBytes(firstExport),
                    Files.readAllBytes(secondExport),
                    "repeated rigid loop bakes must export byte-identically");
        } finally {
            Files.deleteIfExists(firstExport);
            Files.deleteIfExists(secondExport);
        }
    }

    private static BakeResult bakeAndValidate(BoneMapper mapper,
                                              BedrockAnimationData.Animation animation) {
        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setSourceAnimation(animation);
            baker.initialize();
            baker.runToEnd();
            LoopSeamReport report = baker.getLoopSeamReport();
            assertNotNull(report);
            LoopSeamReport diagnostic = baker.isLoopSeamCorrectionRejected()
                    && baker.getBestLoopSeamCandidateReport() != null
                    ? baker.getBestLoopSeamCandidateReport() : report;
            assertFalse(baker.isLoopSeamCorrectionRejected(), seamSummary(diagnostic));
            assertTrue(report.passes(mapper.getConfig()), seamSummary(report));
            assertTrue(report.collisionSafe(), seamSummary(report));
            assertTrue(report.maximumPenetration()
                    <= mapper.getConfig().rigidBodyMaximumSafePenetration);
            baker.requireSafeForExport();
            PhysicsBaker.BakedFrame first = baker.getFrames().get(0);
            PhysicsBaker.BakedFrame last = baker.getFrames().get(
                    baker.getFrames().size() - 1);
            for (String boneName : RIGID_BONES) {
                assertArrayEquals(first.getBoneState(boneName).position,
                        last.getBoneState(boneName).position, 0, boneName + " position");
                assertArrayEquals(first.getBoneState(boneName).rotation,
                        last.getBoneState(boneName).rotation, 0, boneName + " rotation");
            }
            return new BakeResult(List.copyOf(baker.getFrames()), report);
        }
    }

    private static BoneMapper mapper(BedrockModelData.Geometry geometry) {
        BoneMapper mapper = new BoneMapper(geometry.bones);
        RIGID_BONES.forEach(mapper::addPhysicsBone);
        mapper.addCollisionRoot("UpperBody");
        BoneMapper.PhysicsGroupConfig config = mapper.getConfig();
        config.simulationMode = BoneMapper.SimulationMode.RIGID_BODY;
        config.loopMode = BoneMapper.LoopMode.FORCE_LOOP;
        config.loopSeamStrategy = BoneMapper.LoopSeamStrategy.VISUAL_SUBTREE;
        return mapper;
    }

    private static Path propertyPath(String name) {
        String value = System.getProperty(name, "");
        return value.isBlank() ? Path.of("missing-" + name) : Path.of(value);
    }

    private static double[] firstValue(java.util.Map<Double, double[]> values) {
        return values.values().iterator().next();
    }

    private static double[] lastValue(java.util.Map<Double, double[]> values) {
        double[] last = null;
        for (double[] value : values.values()) last = value;
        return last;
    }

    private static String seamSummary(LoopSeamReport report) {
        LoopSeamReport.Metrics relative = report.physicsRelative();
        return "window=" + report.correctionWindowRatio()
                + ", relativeLinear=" + relative.maximumLinearVelocityJump()
                + ", relativeAngular=" + relative.maximumAngularVelocityJump()
                + ", relativeLinearAcceleration="
                + relative.maximumLinearAccelerationJump()
                + ", relativeAngularAcceleration="
                + relative.maximumAngularAccelerationJump()
                + ", quantizedWorld="
                + report.quantizedFinalWorld().maximumPositionError()
                + ", penetration=" + report.maximumPenetration();
    }

    private record BakeResult(List<PhysicsBaker.BakedFrame> frames,
                              LoopSeamReport report) {
    }
}
