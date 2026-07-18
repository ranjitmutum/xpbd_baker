package xpbd.rigidbody;

import org.junit.jupiter.api.Test;
import xpbd.baker.BoneMapper;
import xpbd.baker.BonePoseCalculator;
import xpbd.loader.BedrockAnimationData;
import xpbd.loader.BedrockModelData;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.assertEquals;

final class RigidBodyBakeSessionTest {
    @Test
    void numericAncestorScaleDoesNotBlockRigidBake() {
        BedrockModelData.Bone parent = bone("parent", null, 0);
        BedrockModelData.Bone simulated = bone("simulated", parent.name, 1);
        simulated.cubes.add(cube(0, -0.5, -0.5, 1, 1, 1));

        BedrockAnimationData.Animation animation = new BedrockAnimationData.Animation();
        animation.animationLength = 1;
        BedrockAnimationData.BoneAnimation parentChannel =
                new BedrockAnimationData.BoneAnimation();
        parentChannel.scale = new BedrockAnimationData.Keyframes();
        parentChannel.scale.keyframes.put(0.0, new double[]{1, 1, 1});
        parentChannel.scale.keyframes.put(1.0, new double[]{1, 0.95, 1});
        animation.bones.put(parent.name, parentChannel);

        BoneMapper mapper = rigidMapper(List.of(parent, simulated), simulated.name);
        Map<String, BonePoseCalculator.Pose> initial = BonePoseCalculator.calculate(
                mapper.getAllBones(), animation, 0);
        RecordingBackend backend = new RecordingBackend();

        try (RigidBodyBakeSession session = new RigidBodyBakeSession(
                mapper, animation, initial, backend)) {
            assertEquals(1, session.getPhysicsBodyCount());
        }
    }

    @Test
    void emptyPhysicsBoneUsesCubesFromTwoUnselectedChildGroups() {
        BedrockModelData.Bone owner = bone("owner", null, 0);
        BedrockModelData.Bone left = bone("left_block", owner.name, -2);
        BedrockModelData.Bone right = bone("right_block", owner.name, 2);
        left.cubes.add(cube(-3, -1, -1, 2, 2, 2));
        right.cubes.add(cube(1, -1, -1, 2, 2, 2));

        BoneMapper mapper = rigidMapper(List.of(owner, left, right), owner.name);
        Map<String, BonePoseCalculator.Pose> initial = BonePoseCalculator.calculate(
                mapper.getAllBones(), null, 0);
        RecordingBackend backend = new RecordingBackend();

        try (RigidBodyBakeSession session = new RigidBodyBakeSession(
                mapper, null, initial, backend)) {
            assertEquals(1, session.getPhysicsBodyCount());
            assertEquals(1, backend.definitions.size());
            assertEquals(2, backend.definitions.get(0).boxes().size());
            assertEquals("owner", backend.definitions.get(0).name());
        }
    }

    @Test
    void selectedDescendantKeepsItsCubeOutOfAncestorFallbackBody() {
        BedrockModelData.Bone owner = bone("owner", null, 0);
        BedrockModelData.Bone decoration = bone("decoration", owner.name, -2);
        BedrockModelData.Bone selectedChild = bone("selected_child", owner.name, 2);
        decoration.cubes.add(cube(-3, -1, -1, 2, 2, 2));
        selectedChild.cubes.add(cube(1, -1, -1, 2, 2, 2));

        BoneMapper mapper = rigidMapper(
                List.of(owner, decoration, selectedChild), owner.name, selectedChild.name);
        Map<String, BonePoseCalculator.Pose> initial = BonePoseCalculator.calculate(
                mapper.getAllBones(), null, 0);
        RecordingBackend backend = new RecordingBackend();

        try (RigidBodyBakeSession session = new RigidBodyBakeSession(
                mapper, null, initial, backend)) {
            assertEquals(2, session.getPhysicsBodyCount());
            assertEquals(2, backend.definitions.size());
            assertEquals(1, backend.definitions.get(0).boxes().size());
            assertEquals(1, backend.definitions.get(1).boxes().size());
            assertEquals("owner", backend.definitions.get(0).name());
            assertEquals("selected_child", backend.definitions.get(1).name());
        }
    }

    @Test
    void emptyFixedPhysicsRootAnchorsSelectedChildWithoutCollisionGeometry() {
        BedrockModelData.Bone root = bone("empty_root", null, 0);
        BedrockModelData.Bone child = bone("rigid_child", root.name, 1);
        child.cubes.add(cube(0.5, -0.5, -0.5, 1, 1, 1));

        BoneMapper mapper = rigidMapper(
                List.of(root, child), root.name, child.name);
        Map<String, BonePoseCalculator.Pose> initial = BonePoseCalculator.calculate(
                mapper.getAllBones(), null, 0);
        RecordingBackend backend = new RecordingBackend();

        try (RigidBodyBakeSession session = new RigidBodyBakeSession(
                mapper, null, initial, backend)) {
            assertEquals(2, session.getPhysicsBodyCount());
            assertEquals(2, backend.definitions.size());
            assertEquals("empty_root", backend.definitions.get(0).name());
            assertEquals(RigidBodyBackend.MotionType.KINEMATIC,
                    backend.definitions.get(0).motionType());
            assertEquals(0, backend.definitions.get(0).boxes().size());
            assertEquals("rigid_child", backend.definitions.get(1).name());
            assertEquals(RigidBodyBackend.MotionType.DYNAMIC,
                    backend.definitions.get(1).motionType());
            assertEquals(1, backend.definitions.get(1).boxes().size());
            assertEquals(1, backend.jointCount);
        }
    }

    @Test
    void sweepHitDoesNotOverrideAnimatedKinematicTransform() {
        BedrockModelData.Bone mover = new BedrockModelData.Bone();
        mover.name = "mover";
        mover.pivot = new double[]{0, 0, 0};
        BedrockModelData.Cube cube = new BedrockModelData.Cube();
        cube.origin = new double[]{-0.25, -0.25, -0.25};
        cube.size = new double[]{0.5, 0.5, 0.5};
        mover.cubes.add(cube);

        BoneMapper mapper = new BoneMapper(List.of(mover));
        mapper.addPhysicsBone(mover.name);
        mapper.getConfig().simulationMode = BoneMapper.SimulationMode.RIGID_BODY;
        mapper.getConfig().rigidBodySubsteps = 1;
        mapper.getConfig().rigidBodyUnitScale = 1;

        BedrockAnimationData.Animation animation = new BedrockAnimationData.Animation();
        animation.animationLength = 1;
        BedrockAnimationData.BoneAnimation channel = new BedrockAnimationData.BoneAnimation();
        channel.position = new BedrockAnimationData.Keyframes();
        channel.position.keyframes.put(0.0, new double[]{0, 0, 0});
        channel.position.keyframes.put(1.0, new double[]{4, 0, 0});
        animation.bones.put(mover.name, channel);
        Map<String, BonePoseCalculator.Pose> initial = BonePoseCalculator.calculate(
                mapper.getAllBones(), animation, 0);
        RecordingBackend backend = new RecordingBackend();

        try (RigidBodyBakeSession session = new RigidBodyBakeSession(
                mapper, animation, initial, backend)) {
            session.advance(0, 1, 1, true);

            assertEquals(-4, backend.applied.translation()[0], 1e-9,
                    "a diagnostic sweep must not pin an animated kinematic body at TOI");
            assertEquals(1, session.getSweepHitCount());
        }
    }

    @Test
    void groundPlaneIsCreatedOnlyWhenEnabled() {
        BedrockModelData.Bone body = bone("body", null, 0);
        body.cubes.add(cube(-0.5, 0.5, -0.5, 1, 1, 1));
        BoneMapper mapper = rigidMapper(List.of(body), body.name);
        mapper.getConfig().enableGroundCollision = true;
        Map<String, BonePoseCalculator.Pose> initial = BonePoseCalculator.calculate(
                mapper.getAllBones(), null, 0);
        RecordingBackend backend = new RecordingBackend();

        try (RigidBodyBakeSession session = new RigidBodyBakeSession(
                mapper, null, initial, backend)) {
            assertEquals(1, session.getPhysicsBodyCount());
            assertEquals(RigidBodyBakeSession.GROUND_BODY_NAME, backend.groundName);
            assertEquals(0, backend.groundHeight, 0);
        }
    }

    @Test
    void reusesOuterEndpointPoseAndSamplesOnlyIntermediateSubsteps() {
        BedrockModelData.Bone mover = bone("mover", null, 0);
        mover.cubes.add(cube(-0.5, -0.5, -0.5, 1, 1, 1));
        BoneMapper mapper = rigidMapper(List.of(mover), mover.name);
        mapper.getConfig().rigidBodySubsteps = 4;
        BedrockAnimationData.Animation animation = new BedrockAnimationData.Animation();
        animation.animationLength = 1;
        Map<String, BonePoseCalculator.Pose> initial = BonePoseCalculator.calculate(
                mapper.getAllBones(), animation, 0);
        Map<String, BonePoseCalculator.Pose> endpoint = BonePoseCalculator.calculate(
                mapper.getAllBones(), animation, 1);
        RecordingBackend backend = new RecordingBackend();
        AtomicInteger samples = new AtomicInteger();

        try (RigidBodyBakeSession session = new RigidBodyBakeSession(
                mapper, animation, initial, backend)) {
            session.setPoseSampler(time -> {
                samples.incrementAndGet();
                return BonePoseCalculator.calculate(
                        mapper.getAllBones(), animation, time);
            });
            session.advance(0, 1, 1, true, 0, endpoint);
        }

        assertEquals(3, samples.get(),
                "the fourth substep must reuse the already evaluated endpoint");
    }

    private static BoneMapper rigidMapper(List<BedrockModelData.Bone> bones,
                                          String... physicsBones) {
        BoneMapper mapper = new BoneMapper(bones);
        for (String physicsBone : physicsBones) mapper.addPhysicsBone(physicsBone);
        mapper.getConfig().simulationMode = BoneMapper.SimulationMode.RIGID_BODY;
        mapper.getConfig().rigidBodySubsteps = 1;
        mapper.getConfig().rigidBodyUnitScale = 1;
        return mapper;
    }

    private static BedrockModelData.Bone bone(String name, String parent,
                                               double pivotX) {
        BedrockModelData.Bone bone = new BedrockModelData.Bone();
        bone.name = name;
        bone.parent = parent;
        bone.pivot = new double[]{pivotX, 0, 0};
        return bone;
    }

    private static BedrockModelData.Cube cube(double x, double y, double z,
                                               double sx, double sy, double sz) {
        BedrockModelData.Cube cube = new BedrockModelData.Cube();
        cube.origin = new double[]{x, y, z};
        cube.size = new double[]{sx, sy, sz};
        return cube;
    }

    private static final class RecordingBackend implements RigidBodyBackend {
        private BodyHandle handle;
        private Transform applied = Transform.identity();
        private final List<BodyDefinition> definitions = new ArrayList<>();
        private int jointCount;
        private String groundName;
        private double groundHeight = Double.NaN;

        @Override
        public BodyHandle createBody(BodyDefinition definition) {
            definitions.add(definition);
            handle = new BodyHandle(definitions.size(), definition.name());
            applied = definition.initialBoneTransform();
            return handle;
        }

        @Override
        public BodyHandle createGroundPlane(String name, double height,
                                            double friction, double restitution) {
            groundName = name;
            groundHeight = height;
            handle = new BodyHandle(definitions.size() + 1, name);
            return handle;
        }

        @Override
        public void addSpringJoint(BodyHandle bodyA, BodyHandle bodyB,
                                   Transform worldAnchor, JointSettings settings) {
            jointCount++;
        }

        @Override
        public SweepResult sweepKinematic(BodyHandle body, Transform fromBoneTransform,
                                          Transform toBoneTransform) {
            return new SweepResult(true, 0.25, "target");
        }

        @Override
        public void setKinematicTransform(BodyHandle body, Transform boneTransform,
                                          double dt, boolean continuousHistory) {
            applied = boneTransform;
        }

        @Override
        public void applyCentralForce(BodyHandle body, double[] force) {
        }

        @Override
        public void step(double fixedDt) {
        }

        @Override
        public BodyState getBodyState(BodyHandle body) {
            return new BodyState(applied, new double[3], new double[3]);
        }

        @Override
        public int getContactCount() {
            return 0;
        }

        @Override
        public double getMaximumPenetration() {
            return 0;
        }

        @Override
        public int getNativeBulletVersion() {
            return 287;
        }

        @Override
        public void close() {
        }
    }
}
