package xpbd.baker;

import org.junit.jupiter.api.Test;
import xpbd.loader.BedrockAnimationData;
import xpbd.loader.BedrockModelData;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

final class PhysicsBakerLoopTest {
    @Test
    void usesOneConstantStepAndStopsAfterStableCycles() {
        BoneMapper mapper = fixedMapper();
        mapper.getConfig().minimumWarmupCycles = 2;
        mapper.getConfig().maximumWarmupCycles = 12;
        mapper.getConfig().requiredStableCycles = 2;
        BedrockAnimationData.Animation animation = loopAnimation(0.1);

        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setDt(0.06);
            baker.setSourceAnimation(animation);
            baker.initialize();

            assertEquals(0.05, baker.getCycleDt(), 1e-12);
            assertEquals(24, baker.getTotalSteps());

            baker.runToEnd();

            assertTrue(baker.isLoopConverged());
            assertFalse(baker.isLoopFallbackUsed());
            assertEquals(3, baker.getCompletedLoopCycles());
            assertEquals(6, baker.getCurrentStep());
            assertEquals(6, baker.getTotalSteps());
            assertEquals(3, baker.getFrames().size());
            assertEquals(0, baker.getFrames().get(0).time, 1e-12);
            assertEquals(0.05, baker.getFrames().get(1).time, 1e-12);
            assertEquals(0.1, baker.getFrames().get(2).time, 1e-12);
            assertNotNull(baker.getLoopErrorReport());
            assertEquals(0, baker.getLoopErrorReport().maximumPositionError(), 1e-12);
            assertEquals(0, baker.getLoopErrorReport()
                    .maximumLinearVelocityError(), 1e-12);
        }
    }

    @Test
    void maximumCycleExitIsExplicitAndKeepsBestCompleteCycle() {
        BedrockModelData.Bone falling = new BedrockModelData.Bone();
        falling.name = "falling";
        BoneMapper mapper = new BoneMapper(List.of(falling));
        mapper.addPhysicsBone(falling.name);
        BoneMapper.BonePhysicsConfig boneConfig = new BoneMapper.BonePhysicsConfig();
        boneConfig.fixed = false;
        mapper.setBoneConfig(falling.name, boneConfig);
        mapper.getConfig().gravityY = -10;
        mapper.getConfig().airDrag = 0;
        mapper.getConfig().turbulence = 0;
        mapper.getConfig().animationPullCompliance = 0;
        mapper.getConfig().minimumWarmupCycles = 1;
        mapper.getConfig().maximumWarmupCycles = 2;
        mapper.getConfig().requiredStableCycles = 1;
        mapper.getConfig().loopSeamFallbackEnabled = false;

        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setDt(0.06);
            baker.setSourceAnimation(loopAnimation(0.1));
            baker.initialize();
            baker.runToEnd();

            assertFalse(baker.isLoopConverged());
            assertFalse(baker.isLoopFallbackUsed());
            assertEquals(2, baker.getCompletedLoopCycles());
            assertEquals(1, baker.getSelectedLoopCycle());
            assertEquals(4, baker.getCurrentStep());
            assertEquals(3, baker.getFrames().size());
            assertTrue(baker.getLoopErrorReport().maximumPositionError() > 0);
        }
    }

    @Test
    void fixedDriverWrapDoesNotBecomeAOneFrameVelocitySpike() {
        BoneMapper mapper = fixedMapper();
        mapper.getConfig().minimumWarmupCycles = 1;
        mapper.getConfig().maximumWarmupCycles = 1;
        mapper.getConfig().requiredStableCycles = 1;
        mapper.getConfig().loopSeamFallbackEnabled = false;
        BedrockAnimationData.Animation animation = loopAnimation(0.1);
        BedrockAnimationData.BoneAnimation bone = new BedrockAnimationData.BoneAnimation();
        bone.position = new BedrockAnimationData.Keyframes();
        bone.position.keyframes.put(0.0, new double[]{0, 0, 0});
        bone.position.keyframes.put(0.1, new double[]{10, 0, 0});
        animation.bones.put("anchor", bone);

        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setDt(0.05);
            baker.setSourceAnimation(animation);
            baker.initialize();
            baker.runToEnd();

            PhysicsBaker.BoneState end = baker.getFrames().get(
                    baker.getFrames().size() - 1).getBoneState("anchor");
            assertEquals(0, end.linearVelocity[0], 1e-12,
                    "tail-to-head target displacement must not become velocity");
        }
    }

    @Test
    void fixedDriverUsesPeriodicCentralVelocityAtWrap() {
        BoneMapper mapper = fixedMapper();
        mapper.getConfig().minimumWarmupCycles = 1;
        mapper.getConfig().maximumWarmupCycles = 1;
        mapper.getConfig().requiredStableCycles = 1;
        mapper.getConfig().loopSeamFallbackEnabled = false;
        BedrockAnimationData.Animation animation = loopAnimation(0.1);
        BedrockAnimationData.BoneAnimation bone = new BedrockAnimationData.BoneAnimation();
        bone.position = new BedrockAnimationData.Keyframes();
        bone.position.keyframes.put(0.0, new double[]{0, 0, 0});
        bone.position.keyframes.put(0.025, new double[]{1, 0, 0});
        bone.position.keyframes.put(0.075, new double[]{-1, 0, 0});
        bone.position.keyframes.put(0.1, new double[]{0, 0, 0});
        animation.bones.put("anchor", bone);

        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setDt(0.025);
            baker.setSourceAnimation(animation);
            baker.initialize();
            baker.runToEnd();

            PhysicsBaker.BoneState end = baker.getFrames().get(
                    baker.getFrames().size() - 1).getBoneState("anchor");
            assertEquals(-40, end.linearVelocity[0], 1e-9,
                    "boundary velocity must use samples on both sides of phase zero");
        }
    }

    @Test
    void periodicTurbulenceIsDeterministicAcrossRepeatedBakes() {
        List<PhysicsBaker.BakedFrame> first = bakeTurbulentLoop(dynamicLoopMapper());
        List<PhysicsBaker.BakedFrame> second = bakeTurbulentLoop(dynamicLoopMapper());

        assertEquals(first.size(), second.size());
        for (int frameIndex = 0; frameIndex < first.size(); frameIndex++) {
            PhysicsBaker.BoneState left = first.get(frameIndex).getBoneState("tip");
            PhysicsBaker.BoneState right = second.get(frameIndex).getBoneState("tip");
            assertArrayEquals(left.rotation, right.rotation, 0);
            assertArrayEquals(left.linearVelocity, right.linearVelocity, 0);
            assertArrayEquals(left.worldPosition, right.worldPosition, 0);
        }
    }

    private static BoneMapper fixedMapper() {
        BedrockModelData.Bone anchor = new BedrockModelData.Bone();
        anchor.name = "anchor";
        BoneMapper mapper = new BoneMapper(List.of(anchor));
        mapper.addPhysicsBone(anchor.name);
        mapper.getConfig().gravityY = 0;
        mapper.getConfig().airDrag = 0;
        mapper.getConfig().turbulence = 0;
        return mapper;
    }

    private static BoneMapper dynamicLoopMapper() {
        BedrockModelData.Bone anchor = new BedrockModelData.Bone();
        anchor.name = "anchor";
        BedrockModelData.Bone tip = new BedrockModelData.Bone();
        tip.name = "tip";
        tip.parent = anchor.name;
        tip.pivot = new double[]{0, -1, 0};
        BoneMapper mapper = new BoneMapper(List.of(anchor, tip));
        mapper.addPhysicsBone(anchor.name);
        mapper.addPhysicsBone(tip.name);
        mapper.getConfig().gravityY = 0;
        mapper.getConfig().windSpeed = 0;
        mapper.getConfig().airDrag = 0;
        mapper.getConfig().turbulence = 1.5;
        mapper.getConfig().animationPullCompliance = 0;
        mapper.getConfig().minimumWarmupCycles = 1;
        mapper.getConfig().maximumWarmupCycles = 2;
        mapper.getConfig().requiredStableCycles = 1;
        mapper.getConfig().loopSeamFallbackEnabled = false;
        return mapper;
    }

    private static List<PhysicsBaker.BakedFrame> bakeTurbulentLoop(BoneMapper mapper) {
        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setDt(0.025);
            baker.setSourceAnimation(loopAnimation(0.1));
            baker.initialize();
            baker.runToEnd();
            return List.copyOf(baker.getFrames());
        }
    }

    private static BedrockAnimationData.Animation loopAnimation(double length) {
        BedrockAnimationData.Animation animation = new BedrockAnimationData.Animation();
        animation.animationLength = length;
        animation.loop = true;
        animation.loopBehavior = BedrockAnimationData.Animation.LoopBehavior.LOOP;
        return animation;
    }
}
