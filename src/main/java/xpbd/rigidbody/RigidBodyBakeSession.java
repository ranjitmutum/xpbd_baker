package xpbd.rigidbody;

import models.Vector3;
import xpbd.baker.BakeProfiler;
import xpbd.baker.BoneMapper;
import xpbd.baker.BonePoseCalculator;
import xpbd.baker.PhysicsBaker;
import xpbd.baker.PeriodicStateAdapter;
import xpbd.loader.BedrockAnimationData;
import xpbd.loader.BedrockModelData;

import java.util.ArrayList;
import java.util.ArrayDeque;
import java.util.Collections;
import java.util.Comparator;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/**
 * 为一次烘焙维护一个 Bullet 世界。它刻意独立于 XPBD 引擎，
 * 确保同一物理组不会被两个求解器同时推进。
 */
public final class RigidBodyBakeSession implements AutoCloseable {
    static final String GROUND_BODY_NAME = "__ground__";
    private final BoneMapper boneMapper;
    private final BoneMapper.PhysicsGroupConfig config;
    private final RigidBodyBackend backend;
    private final Map<String, BedrockModelData.Bone> bonesByName = new HashMap<>();
    private final Map<String, List<String>> childrenByBone = new HashMap<>();
    private final Map<String, RigidBodyBackend.BodyHandle> physicsBodies =
            new LinkedHashMap<>();
    private final Map<String, RigidBodyBackend.BodyDefinition> physicsBodyDefinitions =
            new LinkedHashMap<>();
    private final Map<String, RigidBodyBackend.BodyHandle> kinematicBodies =
            new LinkedHashMap<>();
    private final Map<String, RigidBodyBackend.Transform> previousKinematicTransforms =
            new HashMap<>();
    private final List<String> orderedPhysicsBones = new ArrayList<>();
    private final double unitScale;
    private final int substeps;
    private final RigidBodyBackend.SnapshotLevel snapshotLevel;
    private final BakeProfiler profiler;
    private PoseSampler poseSampler;
    private int collisionBodyCount;
    private int sourceCubeCount;
    private int skippedDegenerateCubeCount;
    private int skippedBodyBoneCount;
    private long sweepHitCount;
    private int currentContactCount;
    private double maximumPenetration;
    private List<SubstepSnapshot> latestSubstepSnapshots = List.of();
    private boolean closed;

    public record BoneOutput(String boneName, double[] position, double[] rotation,
                             double[] linearVelocity, double[] worldPosition) {
        public BoneOutput {
            Objects.requireNonNull(boneName, "boneName");
            position = position.clone();
            rotation = rotation.clone();
            linearVelocity = linearVelocity.clone();
            worldPosition = worldPosition.clone();
        }

        @Override
        public double[] position() {
            return position.clone();
        }

        @Override
        public double[] rotation() {
            return rotation.clone();
        }

        @Override
        public double[] linearVelocity() {
            return linearVelocity.clone();
        }

        @Override
        public double[] worldPosition() {
            return worldPosition.clone();
        }
    }

    @FunctionalInterface
    public interface PoseSampler {
        Map<String, BonePoseCalculator.Pose> sample(double time);
    }

    /** 最新输出步中一个固定子步的只读 Bullet 证据。 */
    public record SubstepSnapshot(
            int substepIndex, double sampleTime, double fixedDt,
            List<RigidBodyBackend.ContactSnapshot> contacts,
            List<RigidBodyBackend.BodyShapeSnapshot> bodyShapes,
            List<RigidBodyBackend.NamedBodyState> bodyStates) {
        public SubstepSnapshot {
            if (substepIndex < 0 || !Double.isFinite(sampleTime)
                    || !Double.isFinite(fixedDt) || !(fixedDt > 0)) {
                throw new IllegalArgumentException("invalid rigid-body substep snapshot");
            }
            contacts = List.copyOf(Objects.requireNonNull(contacts, "contacts"));
            bodyShapes = List.copyOf(Objects.requireNonNull(bodyShapes, "bodyShapes"));
            bodyStates = List.copyOf(Objects.requireNonNull(bodyStates, "bodyStates"));
        }
    }

    public static RigidBodyBakeSession create(
            BoneMapper boneMapper, BedrockAnimationData.Animation sourceAnimation,
            Map<String, BonePoseCalculator.Pose> initialPoses) {
        return create(boneMapper, sourceAnimation, initialPoses,
                BakeProfiler.disabled());
    }

    public static RigidBodyBakeSession create(
            BoneMapper boneMapper, BedrockAnimationData.Animation sourceAnimation,
            Map<String, BonePoseCalculator.Pose> initialPoses,
            BakeProfiler profiler) {
        Objects.requireNonNull(boneMapper, "boneMapper");
        Objects.requireNonNull(profiler, "profiler");
        BoneMapper.PhysicsGroupConfig config = boneMapper.getConfig();
        validateConfig(config);
        GdxBulletBackend backend;
        try {
            backend = new GdxBulletBackend(
                    0, config.gravityY * config.rigidBodyUnitScale, 0);
        } catch (RuntimeException | LinkageError error) {
            throw new IllegalArgumentException(
                    "cannot load the Bullet rigid-body backend: " + error.getMessage(), error);
        }
        try {
            return new RigidBodyBakeSession(
                    boneMapper, sourceAnimation, initialPoses, backend, profiler);
        } catch (RuntimeException | LinkageError error) {
            backend.close();
            if (error instanceof IllegalArgumentException illegalArgument) {
                throw illegalArgument;
            }
            throw new IllegalArgumentException(
                    "cannot initialize the rigid-body bake: " + error.getMessage(), error);
        }
    }

    RigidBodyBakeSession(BoneMapper boneMapper,
                         BedrockAnimationData.Animation sourceAnimation,
                         Map<String, BonePoseCalculator.Pose> initialPoses,
                         RigidBodyBackend backend) {
        this(boneMapper, sourceAnimation, initialPoses, backend,
                BakeProfiler.disabled());
    }

    RigidBodyBakeSession(BoneMapper boneMapper,
                         BedrockAnimationData.Animation sourceAnimation,
                         Map<String, BonePoseCalculator.Pose> initialPoses,
                         RigidBodyBackend backend, BakeProfiler profiler) {
        this.boneMapper = Objects.requireNonNull(boneMapper, "boneMapper");
        this.config = boneMapper.getConfig();
        validateConfig(config);
        this.backend = Objects.requireNonNull(backend, "backend");
        this.profiler = Objects.requireNonNull(profiler, "profiler");
        this.snapshotLevel = Objects.requireNonNull(
                config.rigidBodySnapshotLevel, "rigidBodySnapshotLevel");
        backend.setProfiler(profiler);
        backend.setSnapshotLevel(snapshotLevel);
        this.unitScale = config.rigidBodyUnitScale;
        this.substeps = config.rigidBodySubsteps;
        this.poseSampler = time -> BonePoseCalculator.calculate(
                boneMapper.getAllBones(), sourceAnimation, time);
        profiler.setCounter(BakeProfiler.Counter.BONE_COUNT,
                boneMapper.getAllBones().size());
        initialize(Objects.requireNonNull(initialPoses, "initialPoses"));
    }

    private void initialize(Map<String, BonePoseCalculator.Pose> initialPoses) {
        for (BedrockModelData.Bone bone : boneMapper.getAllBones()) {
            if (bone != null && bone.name != null) {
                bonesByName.put(bone.name, bone);
                if (bone.parent != null) {
                    childrenByBone.computeIfAbsent(
                            bone.parent, ignored -> new ArrayList<>()).add(bone.name);
                }
            }
        }
        orderedPhysicsBones.addAll(boneMapper.getPhysicsBones());
        orderedPhysicsBones.sort(Comparator.comparingInt(this::hierarchyDepth));
        if (orderedPhysicsBones.isEmpty()) {
            throw new IllegalArgumentException(
                    "rigid-body mode requires at least one physics bone");
        }

        Set<String> collisionBones = boneMapper.getExpandedCollisionBones();

        for (String boneName : orderedPhysicsBones) {
            BedrockModelData.Bone bone = requireBone(boneName);
            BonePoseCalculator.Pose pose = requirePose(initialPoses, boneName);
            boolean fixed = boneMapper.isFixedBone(boneName);
            RigidBodyBackend.MotionType motionType = fixed
                    ? RigidBodyBackend.MotionType.KINEMATIC
                    : RigidBodyBackend.MotionType.DYNAMIC;
            double mass = fixed ? 0 : boneMapper.getEffectiveMass(boneName);
            if (!fixed && !(mass > 0)) {
                throw new IllegalArgumentException(
                        "dynamic rigid-body bone requires positive mass: " + boneName);
            }
            BedrockRigidBodyCompiler.Compilation compilation = compilePhysicsBone(
                    bone, pose, motionType, mass, initialPoses);
            sourceCubeCount += compilation.sourceCubeCount();
            skippedDegenerateCubeCount += compilation.skippedDegenerateCubeCount();
            RigidBodyBackend.BodyDefinition definition = compilation.body()
                    .orElseGet(() -> createEmptyFixedAnchor(
                            boneName, pose, fixed));
            RigidBodyBackend.BodyHandle handle = backend.createBody(definition);
            physicsBodies.put(boneName, handle);
            physicsBodyDefinitions.put(boneName, definition);
            if (fixed) registerKinematic(boneName, handle, definition.initialBoneTransform());
        }

        for (String boneName : collisionBones) {
            BedrockModelData.Bone bone = requireBone(boneName);
            BonePoseCalculator.Pose pose = requirePose(initialPoses, boneName);
            BedrockRigidBodyCompiler.Compilation compilation = compile(
                    bone, pose, RigidBodyBackend.MotionType.KINEMATIC, 0);
            sourceCubeCount += compilation.sourceCubeCount();
            skippedDegenerateCubeCount += compilation.skippedDegenerateCubeCount();
            if (compilation.body().isEmpty()) {
                skippedBodyBoneCount++;
                continue;
            }
            RigidBodyBackend.BodyDefinition definition = compilation.body().get();
            RigidBodyBackend.BodyHandle handle = backend.createBody(definition);
            registerKinematic(boneName, handle, definition.initialBoneTransform());
            collisionBodyCount++;
        }

        if (config.enableGroundCollision) {
            backend.createGroundPlane(GROUND_BODY_NAME, 0,
                    config.rigidBodyFriction, config.rigidBodyRestitution);
        }

        buildJoints(initialPoses);
    }

    private RigidBodyBackend.BodyDefinition createEmptyFixedAnchor(
            String boneName, BonePoseCalculator.Pose pose, boolean fixed) {
        if (!fixed) {
            throw new IllegalArgumentException(
                    "dynamic rigid-body physics bone has no usable cube: "
                            + boneName);
        }
        return new RigidBodyBackend.BodyDefinition(
                boneName, RigidBodyBackend.MotionType.KINEMATIC, List.of(),
                BedrockPoseConverter.fromPose(pose, unitScale), 0,
                config.rigidBodyFriction, config.rigidBodyRestitution,
                RigidBodyBackend.CcdSettings.disabled());
    }

    private BedrockRigidBodyCompiler.Compilation compile(
            BedrockModelData.Bone bone, BonePoseCalculator.Pose pose,
            RigidBodyBackend.MotionType motionType, double mass) {
        return BedrockRigidBodyCompiler.compile(bone, pose, motionType, mass,
                unitScale, config.rigidBodyFriction,
                config.rigidBodyRestitution, config.rigidBodyCcd);
    }

    private BedrockRigidBodyCompiler.Compilation compilePhysicsBone(
            BedrockModelData.Bone bone, BonePoseCalculator.Pose pose,
            RigidBodyBackend.MotionType motionType, double mass,
            Map<String, BonePoseCalculator.Pose> initialPoses) {
        BedrockRigidBodyCompiler.Compilation direct = compile(
                bone, pose, motionType, mass);
        if (direct.body().isPresent()) return direct;

        List<BedrockRigidBodyCompiler.CubeSource> sources = new ArrayList<>();
        Deque<String> pending = new ArrayDeque<>();
        pending.add(bone.name);
        while (!pending.isEmpty()) {
            String sourceName = pending.removeFirst();
            if (!sourceName.equals(bone.name)
                    && boneMapper.isPhysicsBone(sourceName)) {
                continue;
            }
            BedrockModelData.Bone sourceBone = requireBone(sourceName);
            sources.add(new BedrockRigidBodyCompiler.CubeSource(
                    sourceBone, requirePose(initialPoses, sourceName)));
            pending.addAll(childrenByBone.getOrDefault(
                    sourceName, Collections.emptyList()));
        }
        if (sources.size() == 1) return direct;
        return BedrockRigidBodyCompiler.compileCompound(
                bone, pose, sources, motionType, mass, unitScale,
                config.rigidBodyFriction, config.rigidBodyRestitution,
                config.rigidBodyCcd);
    }

    private void registerKinematic(String boneName,
                                   RigidBodyBackend.BodyHandle handle,
                                   RigidBodyBackend.Transform initialTransform) {
        kinematicBodies.put(boneName, handle);
        previousKinematicTransforms.put(boneName, initialTransform);
    }

    private void buildJoints(Map<String, BonePoseCalculator.Pose> initialPoses) {
        for (String childName : orderedPhysicsBones) {
            BedrockModelData.Bone child = requireBone(childName);
            if (child.parent == null) continue;
            RigidBodyBackend.BodyHandle parentHandle = physicsBodies.get(child.parent);
            RigidBodyBackend.BodyHandle childHandle = physicsBodies.get(childName);
            if (parentHandle == null || childHandle == null) continue;
            if (boneMapper.isFixedBone(child.parent)
                    && boneMapper.isFixedBone(childName)) continue;

            double[] bendDegrees = boneMapper
                    .getEffectiveRigidBodyMaxBendDegrees(childName);
            double limitX = config.enableAngleConstraints
                    ? Math.toRadians(bendDegrees[0]) : Math.PI;
            double limitY = config.enableAngleConstraints
                    ? Math.toRadians(bendDegrees[1]) : Math.PI;
            double limitZ = config.enableAngleConstraints
                    ? Math.toRadians(bendDegrees[2]) : Math.PI;
            double stiffness = config.enableAngleConstraints
                    ? config.rigidBodyJointStiffness : 0;
            RigidBodyBackend.JointSettings settings =
                    new RigidBodyBackend.JointSettings(
                            new double[]{-limitX, -limitY, -limitZ},
                            new double[]{limitX, limitY, limitZ},
                            stiffness, config.rigidBodyJointDamping);
            backend.addSpringJoint(parentHandle, childHandle,
                    BedrockPoseConverter.fromPose(
                            requirePose(initialPoses, childName), unitScale),
                    settings);
        }
    }

    /** 使用配置的固定 Bullet 子步推进一个输出步。 */
    public void advance(double startSampleTime, double endSampleTime,
                        double outputDt, boolean continuousHistory) {
        advance(startSampleTime, endSampleTime, outputDt, continuousHistory, 0);
    }

    public void advance(double startSampleTime, double endSampleTime,
                        double outputDt, boolean continuousHistory,
                        double forcingPeriod) {
        advance(startSampleTime, endSampleTime, outputDt, continuousHistory,
                forcingPeriod, null);
    }

    /**
     * 推进一个输出步，并可复用所属烘焙器已计算的端点姿态；中间子步继续使用采样器。
     */
    public void advance(double startSampleTime, double endSampleTime,
                        double outputDt, boolean continuousHistory,
                        double forcingPeriod,
                        Map<String, BonePoseCalculator.Pose> endpointPoses) {
        requireOpen();
        if (!Double.isFinite(startSampleTime) || !Double.isFinite(endSampleTime)
                || !Double.isFinite(outputDt) || !(outputDt > 0)) {
            throw new IllegalArgumentException("rigid-body step times must be finite");
        }
        double fixedDt = outputDt / substeps;
        List<SubstepSnapshot> snapshots = new ArrayList<>(substeps);
        for (int substep = 0; substep < substeps; substep++) {
            profiler.addCounter(BakeProfiler.Counter.SUBSTEP_COUNT, 1);
            boolean substepContinuous = continuousHistory || substep > 0;
            double sampleTime = interpolatedSampleTime(startSampleTime, endSampleTime,
                    (substep + 1.0) / substeps, continuousHistory, forcingPeriod);
            Map<String, BonePoseCalculator.Pose> poses;
            if (endpointPoses != null && substep == substeps - 1) {
                poses = endpointPoses;
                profiler.addCounter(
                        BakeProfiler.Counter.REUSED_ENDPOINT_POSE_COUNT, 1);
            } else {
                long poseStarted = profiler.start(
                        BakeProfiler.Stage.SUBSTEP_POSE_SAMPLE);
                poses = poseSampler.sample(sampleTime);
                profiler.stop(BakeProfiler.Stage.SUBSTEP_POSE_SAMPLE, poseStarted);
            }
            long kinematicStarted = profiler.start(BakeProfiler.Stage.KINEMATIC_DRIVE);
            driveKinematicBodies(poses, fixedDt, substepContinuous);
            profiler.stop(BakeProfiler.Stage.KINEMATIC_DRIVE, kinematicStarted);
            long forceStarted = profiler.start(BakeProfiler.Stage.DYNAMIC_FORCE_TOTAL);
            applyDynamicForces(poses, sampleTime, forcingPeriod);
            profiler.stop(BakeProfiler.Stage.DYNAMIC_FORCE_TOTAL, forceStarted);
            backend.step(fixedDt);
            currentContactCount = backend.getContactCount();
            maximumPenetration = Math.max(
                    maximumPenetration, backend.getMaximumPenetration());
            List<RigidBodyBackend.ContactSnapshot> contacts =
                    snapshotLevel == RigidBodyBackend.SnapshotLevel.NONE
                            ? List.of() : backend.getContactSnapshots();
            List<RigidBodyBackend.BodyShapeSnapshot> shapes =
                    snapshotLevel == RigidBodyBackend.SnapshotLevel.FULL_DIAGNOSTICS
                            ? backend.getBodyShapeSnapshots() : List.of();
            List<RigidBodyBackend.NamedBodyState> states =
                    snapshotLevel == RigidBodyBackend.SnapshotLevel.FULL_DIAGNOSTICS
                            ? backend.getBodyStateSnapshots() : List.of();
            snapshots.add(new SubstepSnapshot(
                    substep, sampleTime, fixedDt, contacts, shapes, states));
        }
        latestSubstepSnapshots = Collections.unmodifiableList(snapshots);
    }

    /** 切换动画目标而不重建 Bullet 世界或刚体。 */
    public void setPoseSampler(PoseSampler sampler) {
        requireOpen();
        poseSampler = Objects.requireNonNull(sampler, "sampler");
    }

    private static double interpolatedSampleTime(double start, double end,
                                                 double fraction,
                                                 boolean continuousHistory,
                                                 double period) {
        if (!continuousHistory) return end;
        if (end < start && period > 0) {
            double time = start + (end + period - start) * fraction;
            return time >= period - 1e-12 ? 0 : time;
        }
        if (end < start) return end;
        return start + (end - start) * fraction;
    }

    private void driveKinematicBodies(Map<String, BonePoseCalculator.Pose> poses,
                                      double fixedDt, boolean continuousHistory) {
        for (Map.Entry<String, RigidBodyBackend.BodyHandle> entry
                : kinematicBodies.entrySet()) {
            String boneName = entry.getKey();
            long convertStarted = profiler.start(BakeProfiler.Stage.KINEMATIC_CONVERT);
            RigidBodyBackend.Transform next = BedrockPoseConverter.fromPose(
                    requirePose(poses, boneName), unitScale);
            profiler.stop(BakeProfiler.Stage.KINEMATIC_CONVERT, convertStarted);
            RigidBodyBackend.Transform previous =
                    previousKinematicTransforms.getOrDefault(boneName, next);
            if (continuousHistory && transformsDiffer(previous, next)) {
                long sweepStarted = profiler.start(BakeProfiler.Stage.KINEMATIC_SWEEP);
                RigidBodyBackend.SweepResult sweep = backend.sweepKinematic(
                        entry.getValue(), previous, next);
                profiler.stop(BakeProfiler.Stage.KINEMATIC_SWEEP, sweepStarted);
                if (sweep.hit()) {
                    sweepHitCount++;
                }
            }
            long setStarted = profiler.start(BakeProfiler.Stage.KINEMATIC_SET);
            backend.setKinematicTransform(entry.getValue(), next,
                    fixedDt, continuousHistory);
            profiler.stop(BakeProfiler.Stage.KINEMATIC_SET, setStarted);
            previousKinematicTransforms.put(boneName, next);
        }
    }

    private void applyDynamicForces(Map<String, BonePoseCalculator.Pose> poses,
                                    double sampleTime, double forcingPeriod) {
        Vector3 modelAirVelocity = PhysicsBaker.relativeAirVelocity(config);
        Vector3 airVelocity = new Vector3(
                modelAirVelocity.x * unitScale,
                modelAirVelocity.y * unitScale,
                modelAirVelocity.z * unitScale);
        for (String boneName : orderedPhysicsBones) {
            if (boneMapper.isFixedBone(boneName)) continue;
            RigidBodyBackend.BodyHandle handle = physicsBodies.get(boneName);
            long stateStarted = profiler.start(BakeProfiler.Stage.DYNAMIC_STATE_READ);
            RigidBodyBackend.BodyState state = backend.getBodyState(handle);
            profiler.stop(BakeProfiler.Stage.DYNAMIC_STATE_READ, stateStarted);
            long computeStarted = profiler.start(BakeProfiler.Stage.DYNAMIC_FORCE_COMPUTE);
            double mass = boneMapper.getEffectiveMass(boneName);
            double[] velocity = state.linearVelocity();
            double gravityForce = mass * config.gravityY * unitScale
                    * (boneMapper.getEffectiveGravityScale(boneName) - 1.0);
            double windInfluence = boneMapper.getEffectiveWindInfluence(boneName);
            double[] force = new double[]{
                    (airVelocity.x - velocity[0]) * config.airDrag
                            * windInfluence * mass,
                    (airVelocity.y - velocity[1]) * config.airDrag
                            * windInfluence * mass + gravityForce,
                    (airVelocity.z - velocity[2]) * config.airDrag
                            * windInfluence * mass
            };

            double turbulence = config.turbulence * unitScale
                    * boneMapper.getEffectiveTurbulenceInfluence(boneName) * mass;
            if (turbulence > 0) {
                double phase = (boneName.hashCode() & 0xffff) * 0.001;
                double basePhase = forcingPeriod > 0
                        ? 2.0 * Math.PI * wrappedPhase(sampleTime, forcingPeriod)
                        : sampleTime;
                force[0] += Math.sin(basePhase + phase) * turbulence;
                force[1] += Math.sin(basePhase * 2.0 + phase * 1.7) * turbulence;
                force[2] += Math.sin(basePhase * 3.0 + phase * 2.3) * turbulence;
            }

            double pullCompliance = boneMapper.getEffectiveAnimPullCompliance(boneName);
            BonePoseCalculator.Pose reference = poses.get(boneName);
            if (pullCompliance > 0 && reference != null) {
                double stiffness = Math.min(200.0, 1.0 / pullCompliance);
                double damping = Math.min(50.0, 2.0 * Math.sqrt(stiffness));
                double[] target = BedrockPoseConverter.fromPose(
                        reference, unitScale).translation();
                double[] current = state.boneTransform().translation();
                for (int axis = 0; axis < 3; axis++) {
                    force[axis] += mass * (stiffness * (target[axis] - current[axis])
                            - damping * velocity[axis]);
                }
            }
            profiler.stop(BakeProfiler.Stage.DYNAMIC_FORCE_COMPUTE, computeStarted);
            long submitStarted = profiler.start(BakeProfiler.Stage.DYNAMIC_FORCE_SUBMIT);
            backend.applyCentralForce(handle, force);
            profiler.stop(BakeProfiler.Stage.DYNAMIC_FORCE_SUBMIT, submitStarted);
        }
    }

    public List<BoneOutput> captureBoneOutputs(
            Map<String, BonePoseCalculator.Pose> referencePoses) {
        requireOpen();
        Objects.requireNonNull(referencePoses, "referencePoses");
        long started = profiler.start(BakeProfiler.Stage.CAPTURE_BONE_OUTPUTS);
        Map<String, RigidBodyBackend.BodyState> states = new HashMap<>();
        for (Map.Entry<String, RigidBodyBackend.BodyHandle> entry
                : physicsBodies.entrySet()) {
            states.put(entry.getKey(), backend.getBodyState(entry.getValue()));
        }

        List<BoneOutput> result = new ArrayList<>(orderedPhysicsBones.size());
        for (String boneName : orderedPhysicsBones) {
            BedrockModelData.Bone bone = requireBone(boneName);
            RigidBodyBackend.BodyState state = states.get(boneName);
            BedrockModelData.Bone parentBone = bone.parent == null
                    ? null : bonesByName.get(bone.parent);
            RigidBodyBackend.Transform parentTransform = null;
            if (parentBone != null) {
                RigidBodyBackend.BodyState parentState = states.get(parentBone.name);
                if (parentState != null) {
                    parentTransform = parentState.boneTransform();
                } else {
                    parentTransform = BedrockPoseConverter.fromPose(
                            requirePose(referencePoses, parentBone.name), unitScale);
                }
            }
            BedrockPoseConverter.LocalChannels channels =
                    BedrockPoseConverter.toLocalChannels(
                            bone, state.boneTransform(), parentBone,
                            parentTransform, unitScale);
            double[] translation = state.boneTransform().translation();
            double[] velocity = state.linearVelocity();
            result.add(new BoneOutput(boneName,
                    channels.position(), channels.rotation(),
                    new double[]{velocity[0] / unitScale,
                            velocity[1] / unitScale, velocity[2] / unitScale},
                    new double[]{translation[0] / unitScale,
                            translation[1] / unitScale, translation[2] / unitScale}));
        }
        List<BoneOutput> outputs = Collections.unmodifiableList(result);
        profiler.stop(BakeProfiler.Stage.CAPTURE_BONE_OUTPUTS, started);
        return outputs;
    }

    public double[] getCurrentWorldPosition(String boneName) {
        requireOpen();
        RigidBodyBackend.BodyHandle handle = physicsBodies.get(boneName);
        if (handle == null) return new double[0];
        double[] translation = backend.getBodyState(handle)
                .boneTransform().translation();
        return new double[]{translation[0] / unitScale,
                translation[1] / unitScale, translation[2] / unitScale};
    }

    public int getPhysicsBodyCount() {
        return physicsBodies.size();
    }

    /** 每个选定物理刚体实际使用的已编译盒体。 */
    public Map<String, RigidBodyBackend.BodyDefinition> getPhysicsBodyDefinitions() {
        requireOpen();
        return Collections.unmodifiableMap(physicsBodyDefinitions);
    }

    public int getCollisionBodyCount() {
        return collisionBodyCount;
    }

    public int getSourceCubeCount() {
        return sourceCubeCount;
    }

    public int getSkippedDegenerateCubeCount() {
        return skippedDegenerateCubeCount;
    }

    public int getSkippedBodyBoneCount() {
        return skippedBodyBoneCount;
    }

    public long getSweepHitCount() {
        return sweepHitCount;
    }

    public int getCurrentContactCount() {
        return currentContactCount;
    }

    public double getMaximumPenetration() {
        return maximumPenetration;
    }

    public List<SubstepSnapshot> getLatestSubstepSnapshots() {
        requireOpen();
        return latestSubstepSnapshots;
    }

    public PeriodicStateAdapter.Snapshot capturePeriodicSnapshot() {
        requireOpen();
        Map<String, PeriodicStateAdapter.BoneState> states = new LinkedHashMap<>();
        for (Map.Entry<String, RigidBodyBackend.BodyHandle> entry
                : physicsBodies.entrySet()) {
            RigidBodyBackend.BodyState state = backend.getBodyState(entry.getValue());
            double[] position = state.boneTransform().translation();
            double[] linearVelocity = state.linearVelocity();
            for (int axis = 0; axis < 3; axis++) {
                position[axis] /= unitScale;
                linearVelocity[axis] /= unitScale;
            }
            states.put(entry.getKey(), new PeriodicStateAdapter.BoneState(
                    position, state.boneTransform().rotation(), linearVelocity,
                    state.angularVelocity()));
        }
        Set<String> contacts = new LinkedHashSet<>();
        double penetration = 0;
        int anomalies = 0;
        if (!latestSubstepSnapshots.isEmpty()) {
            SubstepSnapshot last = latestSubstepSnapshots.get(
                    latestSubstepSnapshots.size() - 1);
            for (RigidBodyBackend.ContactSnapshot contact : last.contacts()) {
                String first = contact.bodyA().name();
                String second = contact.bodyB().name();
                contacts.add(first.compareTo(second) <= 0
                        ? first + "|" + second : second + "|" + first);
                penetration = Math.max(penetration, contact.penetration());
                if (!Double.isFinite(contact.appliedImpulse())) anomalies++;
            }
        }
        return new PeriodicStateAdapter.Snapshot(
                states, contacts, penetration, anomalies);
    }

    private static double wrappedPhase(double time, double period) {
        double wrapped = time % period;
        if (wrapped < 0) wrapped += period;
        return wrapped / period;
    }

    public int getUnsafeCollisionCount() {
        return maximumPenetration > config.rigidBodyMaximumSafePenetration + 1e-9
                ? 1 : 0;
    }

    public int getNativeBulletVersion() {
        return backend.getNativeBulletVersion();
    }

    @Override
    public void close() {
        if (closed) return;
        closed = true;
        backend.close();
    }

    private int hierarchyDepth(String boneName) {
        int depth = 0;
        Set<String> visited = new HashSet<>();
        BedrockModelData.Bone bone = bonesByName.get(boneName);
        while (bone != null && bone.parent != null && visited.add(bone.name)) {
            depth++;
            bone = bonesByName.get(bone.parent);
        }
        return depth;
    }

    private BedrockModelData.Bone requireBone(String boneName) {
        BedrockModelData.Bone bone = bonesByName.get(boneName);
        if (bone == null) {
            throw new IllegalArgumentException("unknown rigid-body bone: " + boneName);
        }
        return bone;
    }

    private static BonePoseCalculator.Pose requirePose(
            Map<String, BonePoseCalculator.Pose> poses, String boneName) {
        BonePoseCalculator.Pose pose = poses.get(boneName);
        if (pose == null) {
            throw new IllegalArgumentException(
                    "missing rigid-body pose for bone: " + boneName);
        }
        return pose;
    }

    private static boolean transformsDiffer(RigidBodyBackend.Transform a,
                                            RigidBodyBackend.Transform b) {
        double[] at = a.translation();
        double[] bt = b.translation();
        double distanceSquared = 0;
        for (int axis = 0; axis < 3; axis++) {
            double delta = at[axis] - bt[axis];
            distanceSquared += delta * delta;
        }
        double[] ar = a.rotation();
        double[] br = b.rotation();
        double dot = 0;
        for (int axis = 0; axis < 4; axis++) dot += ar[axis] * br[axis];
        return distanceSquared > 1e-16 || Math.abs(dot) < 1.0 - 1e-10;
    }

    private static void validateConfig(BoneMapper.PhysicsGroupConfig config) {
        if (config.rigidBodySubsteps < 1 || config.rigidBodySubsteps > 16) {
            throw new IllegalArgumentException(
                    "rigid-body substeps must be between 1 and 16");
        }
        if (!Double.isFinite(config.rigidBodyUnitScale)
                || !(config.rigidBodyUnitScale > 0)) {
            throw new IllegalArgumentException(
                    "rigid-body unit scale must be finite and positive");
        }
        requireNonNegative(config.rigidBodyJointStiffness, "joint stiffness");
        requireNonNegative(config.rigidBodyJointDamping, "joint damping");
        requireAngle(config.rigidBodyMaxBendXDegrees, "X joint angle");
        requireAngle(config.rigidBodyMaxBendYDegrees, "Y joint angle");
        requireAngle(config.rigidBodyMaxBendZDegrees, "Z joint angle");
        requireNonNegative(config.rigidBodyFriction, "friction");
        requireNonNegative(config.rigidBodyMaximumSafePenetration,
                "maximum safe penetration");
        if (!Double.isFinite(config.rigidBodyRestitution)
                || config.rigidBodyRestitution < 0
                || config.rigidBodyRestitution > 1) {
            throw new IllegalArgumentException(
                    "rigid-body restitution must be between zero and one");
        }
    }

    private static void requireNonNegative(double value, String label) {
        if (!Double.isFinite(value) || value < 0) {
            throw new IllegalArgumentException(
                    "rigid-body " + label + " must be finite and non-negative");
        }
    }

    private static void requireAngle(double value, String label) {
        if (!Double.isFinite(value) || value < 0 || value > 180) {
            throw new IllegalArgumentException(
                    "rigid-body " + label + " must be between 0 and 180 degrees");
        }
    }

    private void requireOpen() {
        if (closed) throw new IllegalStateException("rigid-body bake session is closed");
    }
}
