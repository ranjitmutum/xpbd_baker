package xpbd.rigidbody;

import org.junit.jupiter.api.Test;
import xpbd.baker.BoneMapper;
import xpbd.baker.BonePoseCalculator;
import xpbd.baker.PhysicsBaker;
import xpbd.baker.TransitionBakeRequest;
import xpbd.loader.BedrockAnimationData;
import xpbd.loader.BedrockModelData;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

final class RigidBodyPhysicsBakerIntegrationTest {
    @Test
    void realGravityFieldDisablesAnimationTetherAndAllowsRigidBodyFreeFall() {
        double assisted = bakeTetheredRigidBody(false);
        double realGravity = bakeTetheredRigidBody(true);

        assertTrue(realGravity < assisted - 0.5,
                "real gravity must fall substantially farther than animation-assisted mode: "
                        + realGravity + " vs " + assisted);
        assertTrue(realGravity < -1.0,
                "an unanchored rigid body must continuously accelerate downward: "
                        + realGravity);
    }

    @Test
    void rigidBodyModeAcceptsExplicitTransitionRequest() {
        BedrockModelData.Bone root = boneWithCube(
                "root", null, new double[]{0, 0, 0},
                new double[]{-0.5, -0.5, -0.5}, new double[]{1, 1, 1});
        BoneMapper mapper = new BoneMapper(List.of(root));
        mapper.addPhysicsBone(root.name);
        mapper.getConfig().simulationMode = BoneMapper.SimulationMode.RIGID_BODY;
        mapper.getConfig().rigidBodyUnitScale = 1;
        mapper.getConfig().gravityY = 0;
        mapper.getConfig().windSpeed = 0;
        mapper.getConfig().airDrag = 0;
        mapper.getConfig().turbulence = 0;
        BedrockAnimationData.Animation source = new BedrockAnimationData.Animation();
        source.animationLength = 0.1;
        BedrockAnimationData.Animation target = new BedrockAnimationData.Animation();
        target.animationLength = 0.1;

        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setDt(0.01);
            baker.setSourceAnimation(source);
            baker.setTransitionRequest(new TransitionBakeRequest(
                    source, target, 0.03, 0.02, 0.05, Map.of(root.name, 0.5)));
            baker.initialize();
            baker.runToEnd();

            assertTrue(baker.isRigidBodyMode());
            assertTrue(baker.isTransitionBake());
            assertEquals(0.05, baker.getFrames().get(
                    baker.getFrames().size() - 1).time, 1e-12);
        }
    }

    @Test
    void formalBakerUsesBulletAndKeepsSourceFrameGrid() {
        BedrockModelData.Bone collider = boneWithCube(
                "body", null, new double[]{5, 0, 0},
                new double[]{4.5, -0.5, -0.5}, new double[]{1, 1, 1});
        BedrockModelData.Bone anchor = boneWithCube(
                "anchor", null, new double[]{0, 0, 0},
                new double[]{-0.25, -0.25, -0.25},
                new double[]{0.5, 0.5, 0.5});
        BedrockModelData.Bone ribbon = boneWithCube(
                "ribbon", "anchor", new double[]{0, -1, 0},
                new double[]{-0.2, -2.0, -0.2},
                new double[]{0.4, 1.7, 0.4});
        BoneMapper mapper = new BoneMapper(List.of(collider, anchor, ribbon));
        mapper.addPhysicsBone(anchor.name);
        mapper.addPhysicsBone(ribbon.name);
        mapper.addCollisionRoot(collider.name);
        mapper.getConfig().simulationMode = BoneMapper.SimulationMode.RIGID_BODY;
        mapper.getConfig().rigidBodyUnitScale = 1;
        mapper.getConfig().rigidBodySubsteps = 2;
        mapper.getConfig().rigidBodyMaximumSafePenetration = 10;
        mapper.getConfig().gravityY = -1;
        mapper.getConfig().windSpeed = 0;
        mapper.getConfig().airDrag = 0;
        mapper.getConfig().turbulence = 0;
        mapper.getConfig().animationPullCompliance = 0;
        BedrockAnimationData.Animation animation = thirtyFpsAnimation(anchor.name);

        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setSourceAnimation(animation);
            baker.initialize();

            assertTrue(baker.isRigidBodyMode());
            assertEquals(287, baker.getNativeBulletVersion());
            assertEquals(2, baker.getRigidBodyPhysicsBodyCount());
            assertEquals(1, baker.getBodyColliderCount());
            assertNotNull(baker.getCurrentWorldPosition(ribbon.name));

            baker.runToEnd();

            assertEquals(1.0 / 30.0, baker.getOutputFrameInterval(), 1e-12);
            assertEquals(4, baker.getFrames().size());
            for (int index = 0; index < baker.getFrames().size(); index++) {
                PhysicsBaker.BakedFrame frame = baker.getFrames().get(index);
                assertEquals(index / 30.0, frame.time, 1e-9);
                assertNotNull(frame.getBoneState(anchor.name));
                assertNotNull(frame.getBoneState(ribbon.name));
                assertFinalWorldPositionsMatchChannels(
                        mapper, animation, frame);
            }
        }
    }

    @Test
    void rigidModeRejectsPhysicsBoneWithoutUsableCube() {
        BedrockModelData.Bone empty = new BedrockModelData.Bone();
        empty.name = "empty_anchor";
        BoneMapper mapper = new BoneMapper(List.of(empty));
        mapper.addPhysicsBone(empty.name);
        BoneMapper.BonePhysicsConfig config = new BoneMapper.BonePhysicsConfig();
        config.fixed = false;
        mapper.setBoneConfig(empty.name, config);
        mapper.getConfig().simulationMode = BoneMapper.SimulationMode.RIGID_BODY;

        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            IllegalArgumentException error = assertThrows(
                    IllegalArgumentException.class, baker::initialize);
            assertTrue(error.getMessage().contains("no usable cube"));
        }
    }

    @Test
    void sameSessionTransitionStartsContinuousAndFinalOverlapFailsAudit() {
        BedrockModelData.Bone collider = boneWithCube(
                "body", null, new double[]{0, 0, 0},
                new double[]{-0.5, -0.5, -0.5}, new double[]{1, 1, 1});
        BedrockModelData.Bone mover = boneWithCube(
                "mover", null, new double[]{5, 0, 0},
                new double[]{4.5, -0.5, -0.5}, new double[]{1, 1, 1});
        BoneMapper mapper = new BoneMapper(List.of(collider, mover));
        mapper.addPhysicsBone(mover.name);
        mapper.addCollisionRoot(collider.name);
        mapper.getConfig().simulationMode = BoneMapper.SimulationMode.RIGID_BODY;
        mapper.getConfig().rigidBodyUnitScale = 1;
        mapper.getConfig().gravityY = 0;
        mapper.getConfig().windSpeed = 0;
        mapper.getConfig().airDrag = 0;
        mapper.getConfig().turbulence = 0;
        mapper.getConfig().transitionDuration = 0.05;

        BedrockAnimationData.Animation source = new BedrockAnimationData.Animation();
        source.animationLength = 0.1;
        BedrockAnimationData.Animation transition = new BedrockAnimationData.Animation();
        transition.animationLength = 0.1;
        BedrockAnimationData.BoneAnimation moverTransition =
                new BedrockAnimationData.BoneAnimation();
        moverTransition.position = new BedrockAnimationData.Keyframes();
        moverTransition.position.keyframes.put(0.0, new double[]{-5, 0, 0});
        transition.bones.put(mover.name, moverTransition);

        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setSourceAnimation(source);
            baker.setTransitionAnimation(transition);
            baker.initialize();
            baker.runToEnd();

            PhysicsBaker.BoneState first = baker.getFrames().get(0)
                    .getBoneState(mover.name);
            PhysicsBaker.BoneState last = baker.getFrames().get(
                    baker.getFrames().size() - 1).getBoneState(mover.name);
            assertNotNull(first);
            assertNotNull(last);
            assertEquals(-5, first.worldPosition[0], 1e-6,
                    "the switch frame must preserve A's live physical pose");
            assertTrue(last.worldPosition[0] > first.worldPosition[0],
                    "the inertialized driver must approach B without teleporting");
            assertTrue(baker.getUnsafeFinalCollisionCount() > 0,
                    "shape-level final audit must still reject the eventual overlap");
            assertThrows(IllegalStateException.class, baker::requireSafeForExport);
        }
    }

    @Test
    void descendantCompoundCubeParticipatesInFinalAudit() {
        BedrockModelData.Bone collider = boneWithCube(
                "body", null, new double[]{0, 0, 0},
                new double[]{-0.5, -0.5, -0.5}, new double[]{1, 1, 1});
        BedrockModelData.Bone owner = new BedrockModelData.Bone();
        owner.name = "owner";
        owner.pivot = new double[]{5, 0, 0};
        BedrockModelData.Bone child = boneWithCube(
                "child_shape", owner.name, new double[]{5, 0, 0},
                new double[]{4.5, -0.5, -0.5}, new double[]{1, 1, 1});
        BoneMapper mapper = new BoneMapper(List.of(collider, owner, child));
        mapper.addPhysicsBone(owner.name);
        mapper.addCollisionRoot(collider.name);
        mapper.getConfig().simulationMode = BoneMapper.SimulationMode.RIGID_BODY;
        mapper.getConfig().rigidBodyUnitScale = 1;
        mapper.getConfig().gravityY = 0;
        mapper.getConfig().windSpeed = 0;
        mapper.getConfig().airDrag = 0;
        mapper.getConfig().turbulence = 0;
        mapper.getConfig().transitionDuration = 0.05;

        BedrockAnimationData.Animation source = new BedrockAnimationData.Animation();
        source.animationLength = 0.1;
        BedrockAnimationData.Animation target = new BedrockAnimationData.Animation();
        target.animationLength = 0.1;
        BedrockAnimationData.BoneAnimation ownerTarget =
                new BedrockAnimationData.BoneAnimation();
        ownerTarget.position = new BedrockAnimationData.Keyframes();
        ownerTarget.position.keyframes.put(0.0, new double[]{-5, 0, 0});
        target.bones.put(owner.name, ownerTarget);

        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setSourceAnimation(source);
            baker.setTransitionAnimation(target);
            baker.initialize();
            baker.runToEnd();

            assertTrue(baker.getUnsafeFinalCollisionCount() > 0,
                    "the final audit must use the descendant box compiled into owner");
            assertThrows(IllegalStateException.class, baker::requireSafeForExport);
        }
    }

    @Test
    void dynamicRigidBodyVelocitySurvivesAnimationDriverSwitch() {
        BedrockModelData.Bone falling = boneWithCube(
                "falling", null, new double[]{0, 0, 0},
                new double[]{-0.5, -0.5, -0.5}, new double[]{1, 1, 1});
        BoneMapper mapper = new BoneMapper(List.of(falling));
        mapper.addPhysicsBone(falling.name);
        BoneMapper.BonePhysicsConfig fallingConfig =
                new BoneMapper.BonePhysicsConfig();
        fallingConfig.fixed = false;
        mapper.setBoneConfig(falling.name, fallingConfig);
        mapper.getConfig().simulationMode = BoneMapper.SimulationMode.RIGID_BODY;
        mapper.getConfig().rigidBodyUnitScale = 1;
        mapper.getConfig().rigidBodySubsteps = 2;
        mapper.getConfig().gravityY = -10;
        mapper.getConfig().windSpeed = 0;
        mapper.getConfig().airDrag = 0;
        mapper.getConfig().turbulence = 0;
        mapper.getConfig().animationPullCompliance = 0;
        mapper.getConfig().transitionDuration = 0.1;
        BedrockAnimationData.Animation source = new BedrockAnimationData.Animation();
        source.animationLength = 0.1;
        BedrockAnimationData.Animation target = new BedrockAnimationData.Animation();
        target.animationLength = 0.1;

        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setSourceAnimation(source);
            baker.setTransitionAnimation(target);
            baker.initialize();
            baker.runToEnd();

            PhysicsBaker.BoneState first = baker.getFrames().get(0)
                    .getBoneState(falling.name);
            PhysicsBaker.BoneState next = baker.getFrames().get(1)
                    .getBoneState(falling.name);
            assertTrue(first.linearVelocity[1] < -0.5);
            assertTrue(next.linearVelocity[1] < first.linearVelocity[1]);
            assertTrue(next.worldPosition[1] < first.worldPosition[1]);
        }
    }

    @Test
    void kinematicDriverVelocityContinuesIntoTransition() {
        BedrockModelData.Bone mover = boneWithCube(
                "mover", null, new double[]{0, 0, 0},
                new double[]{-0.5, -0.5, -0.5}, new double[]{1, 1, 1});
        BoneMapper mapper = new BoneMapper(List.of(mover));
        mapper.addPhysicsBone(mover.name);
        mapper.getConfig().simulationMode = BoneMapper.SimulationMode.RIGID_BODY;
        mapper.getConfig().rigidBodyUnitScale = 1;
        mapper.getConfig().rigidBodySubsteps = 1;
        mapper.getConfig().gravityY = 0;
        mapper.getConfig().windSpeed = 0;
        mapper.getConfig().airDrag = 0;
        mapper.getConfig().turbulence = 0;
        mapper.getConfig().transitionDuration = 0.1;

        BedrockAnimationData.Animation source = new BedrockAnimationData.Animation();
        source.animationLength = 0.1;
        BedrockAnimationData.BoneAnimation sourceMover =
                new BedrockAnimationData.BoneAnimation();
        sourceMover.position = new BedrockAnimationData.Keyframes();
        sourceMover.position.keyframes.put(0.0, new double[]{0, 0, 0});
        sourceMover.position.keyframes.put(0.1, new double[]{1, 0, 0});
        source.bones.put(mover.name, sourceMover);

        BedrockAnimationData.Animation target = new BedrockAnimationData.Animation();
        target.animationLength = 0.1;
        BedrockAnimationData.BoneAnimation targetMover =
                new BedrockAnimationData.BoneAnimation();
        targetMover.position = new BedrockAnimationData.Keyframes();
        targetMover.position.keyframes.put(0.0, new double[]{1, 0, 0});
        targetMover.position.keyframes.put(0.1, new double[]{1, 0, 0});
        target.bones.put(mover.name, targetMover);

        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setDt(0.001);
            baker.setSourceAnimation(source);
            baker.setTransitionAnimation(target);
            baker.initialize();
            baker.runToEnd();

            PhysicsBaker.BoneState first = baker.getFrames().get(0)
                    .getBoneState(mover.name);
            PhysicsBaker.BoneState next = baker.getFrames().get(1)
                    .getBoneState(mover.name);
            assertEquals(-10, first.linearVelocity[0], 2e-5);
            assertTrue(next.linearVelocity[0] < -8,
                    "the first B target step must retain A's forward velocity");
        }
    }

    private static BedrockModelData.Bone boneWithCube(
            String name, String parent, double[] pivot,
            double[] origin, double[] size) {
        BedrockModelData.Bone bone = new BedrockModelData.Bone();
        bone.name = name;
        bone.parent = parent;
        bone.pivot = pivot;
        BedrockModelData.Cube cube = new BedrockModelData.Cube();
        cube.origin = origin;
        cube.size = size;
        bone.cubes.add(cube);
        return bone;
    }

    private static double bakeTetheredRigidBody(boolean realGravityField) {
        BedrockModelData.Bone falling = boneWithCube(
                "real_gravity_rigid", null, new double[]{0, 0, 0},
                new double[]{-0.25, -0.25, -0.25},
                new double[]{0.5, 0.5, 0.5});
        BoneMapper mapper = new BoneMapper(List.of(falling));
        mapper.addPhysicsBone(falling.name);
        mapper.getConfig().simulationMode = BoneMapper.SimulationMode.RIGID_BODY;
        mapper.getConfig().rigidBodyUnitScale = 1;
        mapper.getConfig().enableRealGravityField = realGravityField;
        mapper.getConfig().gravityY = -10;
        mapper.getConfig().windSpeed = 0;
        mapper.getConfig().airDrag = 0;
        mapper.getConfig().turbulence = 0;
        mapper.getConfig().animationPullCompliance = 0.000001;
        mapper.getConfig().rigidBodyMaximumSafePenetration = 10;
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

    private static BedrockAnimationData.Animation thirtyFpsAnimation(
            String anchorName) {
        BedrockAnimationData.Animation animation =
                new BedrockAnimationData.Animation();
        animation.animationLength = 0.1;
        BedrockAnimationData.BoneAnimation anchor =
                new BedrockAnimationData.BoneAnimation();
        anchor.position = new BedrockAnimationData.Keyframes();
        anchor.position.keyframes.put(0.0, new double[]{0, 0, 0});
        anchor.position.keyframes.put(1.0 / 30.0, new double[]{0.1, 0, 0});
        anchor.position.keyframes.put(2.0 / 30.0, new double[]{0.2, 0, 0});
        anchor.position.keyframes.put(0.1, new double[]{0.3, 0, 0});
        animation.bones.put(anchorName, anchor);
        return animation;
    }

    private static void assertFinalWorldPositionsMatchChannels(
            BoneMapper mapper, BedrockAnimationData.Animation animation,
            PhysicsBaker.BakedFrame frame) {
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
                    state.worldPosition, 2e-5);
        }
    }
}
