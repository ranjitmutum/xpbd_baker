package xpbd.regression;

import org.junit.jupiter.api.Test;
import xpbd.baker.BoneMapper;
import xpbd.baker.BonePoseCalculator;
import xpbd.baker.PhysicsBaker;
import xpbd.loader.BedrockAnimationData;
import xpbd.loader.BedrockModelData;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

final class PhysicsBakerCollisionTest {
    @Test
    void collisionRootsAreIndependentExpandedAndClearedWithModelState() {
        BoneMapper mapper = mapperWithBodyAndChain();
        mapper.addCollisionRoot("body");
        mapper.addCollisionRoot("anchor");

        assertTrue(mapper.getExpandedCollisionBones().contains("body"));
        assertFalse(mapper.getExpandedCollisionBones().contains("anchor"));
        assertFalse(mapper.getExpandedCollisionBones().contains("tip"));

        mapper.resetModelState();

        assertTrue(mapper.getCollisionRoots().isEmpty());
        assertTrue(mapper.getPhysicsBones().isEmpty());
    }

    @Test
    void frameZeroEmbeddingIsProjectedBeforeItIsRecorded() {
        BoneMapper mapper = mapperWithBodyAndChain();
        mapper.addCollisionRoot("body");
        mapper.getConfig().gravityY = 0;
        mapper.getConfig().airDrag = 0;
        mapper.getConfig().turbulence = 0;
        mapper.getConfig().animationPullCompliance = 0;
        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.initialize();

            assertEquals(1, baker.getBodyColliderCount());
            assertTrue(baker.getCurrentWorldPosition("tip")[0] < -1.0999);
            PhysicsBaker.BoneState recorded = baker.getFrames().get(0).getBoneState("tip");
            assertNotNull(recorded);
            assertTrue(recorded.worldPosition[0] < -1.0999);
            assertEquals(1, baker.getCollisionDiagnostics().initialEmbedded);
        }
    }

    @Test
    void collisionInitializationRejectsScaleAndAcceptsInflate() {
        BoneMapper scaledMapper = mapperWithBodyAndChain();
        scaledMapper.addCollisionRoot("body");
        BedrockAnimationData.Animation animation = shortAnimation();
        BedrockAnimationData.BoneAnimation bodyChannel = new BedrockAnimationData.BoneAnimation();
        bodyChannel.scale = channel(2, 1, 1);
        animation.bones.put("body", bodyChannel);
        try (PhysicsBaker scaledBaker = new PhysicsBaker(scaledMapper)) {
            scaledBaker.setSourceAnimation(animation);
            assertThrows(IllegalArgumentException.class, scaledBaker::initialize);
        }

        BoneMapper inflatedMapper = mapperWithBodyAndChain();
        inflatedMapper.addCollisionRoot("body");
        inflatedMapper.getAllBones().get(0).cubes.get(0).inflate = 0.1;
        try (PhysicsBaker inflatedBaker = new PhysicsBaker(inflatedMapper)) {
            inflatedBaker.initialize();
            assertEquals(1, inflatedBaker.getBodyColliderCount());
            assertTrue(inflatedBaker.getCurrentWorldPosition("tip")[0] < -1.1999);
        }
    }

    @Test
    void finalizedWorldPositionsMatchFinalLocalChannels() {
        BoneMapper mapper = mapperWithBodyAndChain();
        BedrockAnimationData.Animation animation = shortAnimation();
        BedrockAnimationData.Animation transition = shortAnimation();
        BedrockAnimationData.BoneAnimation tipTransition = new BedrockAnimationData.BoneAnimation();
        tipTransition.position = channel(0.25, 0, 0);
        transition.bones.put("tip", tipTransition);
        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setSourceAnimation(animation);
            baker.setTransitionAnimation(transition);

            baker.initialize();
            baker.runToEnd();

            for (PhysicsBaker.BakedFrame frame : baker.getFrames()) {
            Map<String, double[]> positionOverrides = new HashMap<>();
            Map<String, double[]> rotationOverrides = new HashMap<>();
            for (PhysicsBaker.BoneState state : frame.boneStates) {
                positionOverrides.put(state.boneName, state.position);
                rotationOverrides.put(state.boneName, state.rotation);
            }
            Map<String, BonePoseCalculator.Pose> poses = BonePoseCalculator.calculate(
                    mapper.getAllBones(), animation, frame.time,
                    positionOverrides, rotationOverrides);
            for (PhysicsBaker.BoneState state : frame.boneStates) {
                assertArrayEquals(poses.get(state.boneName).worldPosition,
                        state.worldPosition, 1e-9);
            }
            }
        }
    }

    @Test
    void animationShorterThanFixedStepIntegratesOnlyItsOwnDuration() {
        BedrockModelData.Bone falling = new BedrockModelData.Bone();
        falling.name = "falling";
        falling.pivot = new double[]{0, 0, 0};
        BoneMapper mapper = new BoneMapper(List.of(falling));
        mapper.addPhysicsBone(falling.name);
        BoneMapper.BonePhysicsConfig fallingConfig =
                new BoneMapper.BonePhysicsConfig();
        fallingConfig.fixed = false;
        mapper.setBoneConfig(falling.name, fallingConfig);
        mapper.getConfig().gravityY = -10;
        mapper.getConfig().airDrag = 0;
        mapper.getConfig().turbulence = 0;
        mapper.getConfig().animationPullCompliance = 0;
        BedrockAnimationData.Animation animation = new BedrockAnimationData.Animation();
        animation.animationLength = 0.001;

        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setSourceAnimation(animation);
            baker.initialize();
            baker.runToEnd();

            double y = baker.getCurrentWorldPosition(falling.name)[1];
            assertTrue(y < 0);
            assertTrue(y > -0.0001,
                    "0.001 seconds must not integrate a full 1/60-second step: " + y);
            assertEquals(0.001, baker.getCurrentSampleTime(), 1e-12);
            assertEquals(0.001, baker.getFrames().get(
                    baker.getFrames().size() - 1).time, 1e-12);
        }
    }

    @Test
    void xpbdGroundCollisionIsDisabledByDefaultAndStopsAtConfiguredSkinWhenEnabled() {
        double withoutGround = bakeFallingParticle(false);
        double withGround = bakeFallingParticle(true);

        assertTrue(withoutGround < 0, "compatibility default must still allow falling below Y=0");
        assertEquals(0.1, withGround, 1e-9);
    }

    @Test
    void realGravityFieldDisablesAnimationTetherAndAllowsXpbdFreeFall() {
        double assisted = bakeAnimationTetheredParticle(false);
        double realGravity = bakeAnimationTetheredParticle(true);

        assertTrue(realGravity < assisted - 0.5,
                "real gravity must fall substantially farther than animation-assisted mode: "
                        + realGravity + " vs " + assisted);
        assertTrue(realGravity < -1.0,
                "a free particle must continuously accelerate downward: " + realGravity);
    }

    @Test
    void collisionDisturbedTransitionStillEndsOnReferenceAfterColliderLeaves() {
        BoneMapper mapper = mapperWithBodyAndChain();
        mapper.getAllBones().stream()
                .filter(bone -> "tip".equals(bone.name))
                .findFirst().orElseThrow().pivot = new double[]{0, 0.8, 0};
        mapper.addCollisionRoot("body");
        mapper.getConfig().gravityY = 0;
        mapper.getConfig().airDrag = 0;
        mapper.getConfig().turbulence = 0;
        mapper.getConfig().animationPullCompliance = 0;
        mapper.getConfig().transitionDuration = 0.1;

        BedrockAnimationData.Animation source = new BedrockAnimationData.Animation();
        source.animationLength = 0.05;
        BedrockAnimationData.Animation target = new BedrockAnimationData.Animation();
        target.animationLength = 0.1;
        BedrockAnimationData.BoneAnimation bodyTarget =
                new BedrockAnimationData.BoneAnimation();
        bodyTarget.position = channel(10, 0, 0);
        target.bones.put("body", bodyTarget);

        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setDt(0.01);
            baker.setSourceAnimation(source);
            baker.setTransitionAnimation(target);
            baker.initialize();
            baker.runToEnd();

            PhysicsBaker.BoneState first = baker.getFrames().get(0)
                    .getBoneState("anchor");
            PhysicsBaker.BoneState last = baker.getFrames().get(
                    baker.getFrames().size() - 1).getBoneState("anchor");
            assertEquals(1, baker.getCollisionDiagnostics().initialEmbedded);
            assertTrue(Math.abs(first.rotation[2]) > 0.1,
                    "collision must visibly disturb A's exit pose");
            assertArrayEquals(new double[]{0, 0, 0}, last.rotation, 1e-9,
                    "after B moves the collider away, the clip must end exactly on B");
        }
    }

    private static BoneMapper mapperWithBodyAndChain() {
        BedrockModelData.Bone body = new BedrockModelData.Bone();
        body.name = "body";
        BedrockModelData.Cube cube = new BedrockModelData.Cube();
        cube.origin = new double[]{-1, -1, -1};
        cube.size = new double[]{2, 2, 2};
        body.cubes.add(cube);

        BedrockModelData.Bone anchor = new BedrockModelData.Bone();
        anchor.name = "anchor";
        anchor.pivot = new double[]{-2, 0, 0};

        BedrockModelData.Bone tip = new BedrockModelData.Bone();
        tip.name = "tip";
        tip.parent = "anchor";
        tip.pivot = new double[]{0, 0, 0};

        BoneMapper mapper = new BoneMapper(List.of(body, anchor, tip));
        mapper.addPhysicsBone("anchor");
        mapper.addPhysicsBone("tip");
        return mapper;
    }

    private static double bakeFallingParticle(boolean groundEnabled) {
        BedrockModelData.Bone falling = new BedrockModelData.Bone();
        falling.name = "falling_ground_test";
        falling.pivot = new double[]{0, 0.25, 0};
        BoneMapper mapper = new BoneMapper(List.of(falling));
        mapper.addPhysicsBone(falling.name);
        BoneMapper.BonePhysicsConfig config = new BoneMapper.BonePhysicsConfig();
        config.fixed = false;
        mapper.setBoneConfig(falling.name, config);
        mapper.getConfig().enableGroundCollision = groundEnabled;
        mapper.getConfig().collisionSkin = 0.1;
        mapper.getConfig().gravityY = -10;
        mapper.getConfig().airDrag = 0;
        mapper.getConfig().turbulence = 0;
        mapper.getConfig().animationPullCompliance = 0;
        BedrockAnimationData.Animation animation = new BedrockAnimationData.Animation();
        animation.animationLength = 0.5;

        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setDt(0.01);
            baker.setSourceAnimation(animation);
            baker.initialize();
            baker.runToEnd();
            return baker.getCurrentWorldPosition(falling.name)[1];
        }
    }

    private static double bakeAnimationTetheredParticle(boolean realGravityField) {
        BedrockModelData.Bone falling = new BedrockModelData.Bone();
        falling.name = "real_gravity_xpbd";
        falling.pivot = new double[]{0, 0, 0};
        BoneMapper mapper = new BoneMapper(List.of(falling));
        mapper.addPhysicsBone(falling.name);
        mapper.getConfig().enableRealGravityField = realGravityField;
        mapper.getConfig().gravityY = -10;
        mapper.getConfig().airDrag = 0;
        mapper.getConfig().turbulence = 0;
        mapper.getConfig().animationPullCompliance = 0.000001;
        assertEquals(!realGravityField, mapper.isFixedBone(falling.name),
                "real gravity must release legacy automatic root pinning");
        BedrockAnimationData.Animation animation = new BedrockAnimationData.Animation();
        animation.animationLength = 0.5;

        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setDt(0.01);
            baker.setSourceAnimation(animation);
            baker.initialize();
            baker.runToEnd();
            return baker.getCurrentWorldPosition(falling.name)[1];
        }
    }

    private static BedrockAnimationData.Animation shortAnimation() {
        BedrockAnimationData.Animation animation = new BedrockAnimationData.Animation();
        animation.animationLength = 1.0 / 30.0;
        return animation;
    }

    private static BedrockAnimationData.Keyframes channel(double x, double y, double z) {
        BedrockAnimationData.Keyframes channel = new BedrockAnimationData.Keyframes();
        channel.keyframes.put(0.0, new double[]{x, y, z});
        return channel;
    }
}
