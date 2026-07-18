package xpbd.regression;

import org.junit.jupiter.api.Test;
import xpbd.baker.BoneMapper;
import xpbd.baker.PhysicsBaker;
import xpbd.export.AnimationExporter;
import xpbd.loader.AnimationLoader;
import xpbd.loader.BedrockAnimationData;
import xpbd.loader.BedrockModelData;
import xpbd.loader.ModelLoader;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

/** Opt-in smoke test for the user-supplied huilin model and real gravity mode. */
final class RealModelGravityFieldTest {
    private static final List<String> CHAIN = List.of(
            "bone106", "bone107", "bone108", "bone109");

    @Test
    void allSuppliedHuilinGeometryAndAnimationFilesParse() throws Exception {
        Path modelPath = suppliedModelPath();
        Path packRoot = modelPath.getParent().getParent();
        List<Path> geometryFiles;
        List<Path> animationFiles;
        try (var paths = Files.list(packRoot.resolve("models"))) {
            geometryFiles = paths.filter(Files::isRegularFile)
                    .filter(path -> path.getFileName().toString().endsWith(".json"))
                    .sorted().toList();
        }
        try (var paths = Files.list(packRoot.resolve("animations"))) {
            animationFiles = paths.filter(Files::isRegularFile)
                    .filter(path -> path.getFileName().toString()
                            .endsWith(".animation.json"))
                    .sorted().toList();
        }

        assertFalse(geometryFiles.isEmpty(), "supplied pack has no geometry JSON");
        assertFalse(animationFiles.isEmpty(), "supplied pack has no animation JSON");
        List<String> parsed = new ArrayList<>();
        for (Path geometryFile : geometryFiles) {
            assertFalse(ModelLoader.load(geometryFile.toString()).bones.isEmpty(),
                    "empty geometry: " + geometryFile);
            parsed.add(geometryFile.getFileName().toString());
        }
        for (Path animationFile : animationFiles) {
            assertFalse(AnimationLoader.load(animationFile.toString())
                            .animations.isEmpty(),
                    "empty animation file: " + animationFile);
            parsed.add(animationFile.getFileName().toString());
        }
        assertEquals(7, parsed.size(),
                "expected both models and all five outer animation files");
    }

    @Test
    void suppliedHuilinChainFallsAndSettlesOnGroundInBothSolvers()
            throws Exception {
        Path modelPath = suppliedModelPath();
        Path animationPath = modelPath.getParent().getParent()
                .resolve("animations").resolve("main.animation.json");
        assumeTrue(Files.isRegularFile(modelPath) && Files.isRegularFile(animationPath),
                "gravity smoke requires models/main.json and animations/main.animation.json");

        BedrockModelData.Geometry geometry = ModelLoader.load(modelPath.toString());
        BedrockAnimationData.Animation walk = AnimationLoader.load(
                animationPath.toString()).animations.get("walk");
        assertTrue(walk != null && walk.animationLength > 0,
                "supplied animation file must contain a non-empty walk clip");

        for (BoneMapper.SimulationMode mode : BoneMapper.SimulationMode.values()) {
            BoneMapper mapper = new BoneMapper(geometry.bones);
            CHAIN.forEach(mapper::addPhysicsBone);
            BoneMapper.PhysicsGroupConfig config = mapper.getConfig();
            config.simulationMode = mode;
            config.enableRealGravityField = true;
            config.enableGroundCollision = true;
            config.gravityY = -9.8;
            config.windSpeed = 0;
            config.airDrag = 0;
            config.turbulence = 0;
            config.animationPullCompliance = 0.000001;
            config.rigidBodyMaximumSafePenetration = 10;

            assertFalse(mapper.isFixedBone(CHAIN.get(0)),
                    mode + " must release the legacy automatic root anchor");
            assertEquals(0, mapper.getEffectiveAnimPullCompliance(CHAIN.get(0)), 0,
                    mode + " must ignore even a strong animation tether");

            try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
                baker.setDt(1.0 / 60.0);
                baker.setSourceAnimation(walk);
                baker.initialize();
                double initialY = baker.getCurrentWorldPosition(CHAIN.get(0))[1];

                baker.runSteps(300);

                double finalY = baker.getCurrentWorldPosition(CHAIN.get(0))[1];
                assertTrue(Double.isFinite(finalY),
                        mode + " produced a non-finite root position");
                assertTrue(finalY < initialY - 5,
                        mode + " did not visibly fall: " + initialY + " -> " + finalY);
                assertTrue(finalY > -5,
                        mode + " passed through the enabled Y=0 ground: " + finalY);
                assertTrue(baker.getCurrentStep() >= 120,
                        mode + " stopped before a meaningful real-model simulation");

                baker.runToEnd();
                baker.requireSafeForExport();
                Path exported = Files.createTempFile(
                        "huilin-real-gravity-" + mode.name().toLowerCase(),
                        ".animation.json");
                try {
                    String animationId = "animation.huilin.real_gravity."
                            + mode.name().toLowerCase();
                    AnimationExporter.export(animationId, walk, baker.getFrames(),
                            baker.getOutputLoopBehavior(), exported.toString());
                    BedrockAnimationData.Animation reloaded = AnimationLoader.load(
                            exported.toString()).animations.get(animationId);
                    assertNotNull(reloaded, mode + " export did not reload");
                    for (String boneName : CHAIN) {
                        assertTrue(reloaded.bones.containsKey(boneName),
                                mode + " export omitted " + boneName);
                    }
                } finally {
                    Files.deleteIfExists(exported);
                }
            }
        }
    }

    private static Path suppliedModelPath() {
        String modelProperty = System.getProperty("xpbd.test.gravityModel");
        assumeTrue(modelProperty != null && !modelProperty.isBlank(),
                "set -Dxpbd.test.gravityModel to run the external gravity smoke");
        return Path.of(modelProperty).toAbsolutePath().normalize();
    }
}
