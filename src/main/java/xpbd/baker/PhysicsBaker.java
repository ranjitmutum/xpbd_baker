package xpbd.baker;

import constraints.CrossSpringConstraint;
import constraints.DistanceConstraint;
import constraints.GroundCollisionConstraint;
import constraints.TargetConstraint;
import constraints.VertexFaceCollisionConstraint;
import constraints.WeldConstraint;
import core.XPBDEngine;
import models.Particle;
import models.Vector3;
import xpbd.loader.BedrockAnimationData;
import xpbd.loader.BedrockModelData;
import xpbd.rigidbody.RigidBodyBakeSession;
import xpbd.rigidbody.RigidBodyPeriodicStateAdapter;
import java.util.*;

public final class PhysicsBaker implements AutoCloseable {
    /** 默认 60 Hz 下的十分钟上限，包含自适应循环预热周期。 */
    private static final int MAX_BAKE_STEPS = 36_000;
    static final String DISABLE_ENDPOINT_REUSE_PROPERTY =
            "xpbd.bake.disableEndpointPoseReuse";
    static final String DISABLE_COMPILED_POSES_PROPERTY =
            "xpbd.bake.disableCompiledPoseEvaluator";
    private final BoneMapper boneMapper;
    private final XPBDEngine engine;
    private final List<BakedFrame> frames = new ArrayList<>();
    private final List<BakedFrame> framesView = Collections.unmodifiableList(frames);
    private final Map<String, BedrockModelData.Bone> bonesByName = new HashMap<>();
    private final Map<String, List<String>> physicsChildrenByBone = new HashMap<>();
    private final List<String> orderedPhysicsBoneCache = new ArrayList<>();
    private BonePoseCalculator.Evaluator poseEvaluator;
    private boolean endpointPoseReuseEnabled;
    private boolean compiledPoseEvaluatorEnabled;
    private Map<String, BonePoseCalculator.Pose> currentReferencePoses = Collections.emptyMap();
    private BedrockAnimationData.Animation sourceAnimation;
    private BedrockAnimationData.Animation transitionAnimation;
    private TransitionBakeRequest requestedTransition;
    private TransitionBakeRequest activeTransition;
    private TransitionBakeController transitionController;
    private int transitionPreRollSteps;
    private int transitionSteps;
    private double transitionPreRollDt;
    private double transitionDt;
    private boolean transitionStarted;
    private double dt = 1.0 / 60.0;
    private double cycleDt = dt;
    private double outputFrameInterval = dt;
    private int totalSteps;
    private int currentStep = 0;
    private double currentSampleTime = 0;
    private boolean framesFinalized = false;
    private boolean initialized = false;
    private TargetConstraint[] animationTargets = new TargetConstraint[0];
    private BodyColliderCache bodyColliderCache;
    private VertexFaceCollisionConstraint collisionConstraint;
    private GroundCollisionConstraint groundCollisionConstraint;
    private Particle[] collisionParticles = new Particle[0];
    private int unsafeFinalCollisionCount;
    private RigidBodyBakeSession rigidBodySession;
    private RigidBodyCollisionAuditor rigidBodyCollisionAuditor;
    private double maximumFinalRigidBodyPenetration;
    private final List<BakedFrame> loopCycleFrames = new ArrayList<>();
    private final List<BakedFrame> bestLoopCycleFrames = new ArrayList<>();
    private LoopBakeController loopController;
    private LoopBakeConfig loopBakeConfig;
    private LoopErrorReport loopErrorReport;
    private LoopSeamReport loopSeamReport;
    private LoopSeamReport bestLoopSeamCandidateReport;
    private boolean loopConverged;
    private boolean loopFallbackUsed;
    private boolean loopSeamCorrectionRejected;
    private int completedLoopCycles;
    private LoopCycleCandidate bestLoopCycleCandidate;
    private BakeProfiler profiler = BakeProfiler.disabled();

    public PhysicsBaker(BoneMapper boneMapper) {
        this.boneMapper = boneMapper;
        this.engine = new XPBDEngine();
    }

    public void setSourceAnimation(BedrockAnimationData.Animation anim) {
        this.sourceAnimation = anim;
    }

    public void setTransitionAnimation(BedrockAnimationData.Animation animation) {
        this.transitionAnimation = animation;
    }

    public void setTransitionRequest(TransitionBakeRequest request) {
        if (initialized) {
            throw new IllegalStateException(
                    "transition request cannot change after initialization");
        }
        requestedTransition = request;
    }

    public void setProfiler(BakeProfiler profiler) {
        if (initialized) {
            throw new IllegalStateException(
                    "profiler cannot change after initialization");
        }
        this.profiler = Objects.requireNonNull(profiler, "profiler");
    }

    public BakeProfiler getProfiler() {
        return profiler;
    }

    public void setDt(double dt) {
        if (!Double.isFinite(dt) || dt <= 0) {
            throw new IllegalArgumentException("dt must be a finite value greater than 0");
        }
        if (initialized) {
            throw new IllegalStateException("dt cannot change after initialization; re-create or reset the baker first");
        }
        this.dt = dt;
    }

    /** 循环动画周期内实际使用的固定模拟步长。 */
    public double getCycleDt() {
        return cycleDt;
    }

    /** 最终烘焙的标称间隔（可能遵循源时间线）。 */
    public double getOutputFrameInterval() {
        return outputFrameInterval;
    }

    public List<BakedFrame> getFrames() {
        return framesView;
    }

    public int getCurrentStep() {
        return currentStep;
    }

    public int getTotalSteps() {
        return totalSteps;
    }

    /** 生成当前显示状态时使用的精确源动画时间。 */
    public double getCurrentSampleTime() {
        return currentSampleTime;
    }

    public void initialize() {
        long initializeStarted = profiler.start(BakeProfiler.Stage.INITIALIZE);
        initialized = false;
        closeRigidBodySession();
        frames.clear();
        loopCycleFrames.clear();
        bestLoopCycleFrames.clear();
        currentStep = 0;
        currentSampleTime = 0;
        framesFinalized = false;
        outputFrameInterval = dt;

        configureTransition();
        configureLoopTiming();

        totalSteps = calculateTotalSteps();

        engine.clear();
        animationTargets = new TargetConstraint[0];
        bodyColliderCache = null;
        collisionConstraint = null;
        groundCollisionConstraint = null;
        collisionParticles = new Particle[0];
        unsafeFinalCollisionCount = 0;
        rigidBodyCollisionAuditor = null;
        maximumFinalRigidBodyPenetration = 0;
        loopController = null;
        loopErrorReport = null;
        loopSeamReport = null;
        bestLoopSeamCandidateReport = null;
        loopConverged = false;
        loopFallbackUsed = false;
        loopSeamCorrectionRejected = false;
        completedLoopCycles = 0;
        bestLoopCycleCandidate = null;

        boneMapper.buildParticleMapping();
        rebuildStructureCaches();
        BoneMapper.PhysicsGroupConfig cfg = boneMapper.getConfig();
        endpointPoseReuseEnabled = !Boolean.getBoolean(
                DISABLE_ENDPOINT_REUSE_PROPERTY);
        compiledPoseEvaluatorEnabled = !Boolean.getBoolean(
                DISABLE_COMPILED_POSES_PROPERTY);

        engine.setGravity(new Vector3(0, cfg.gravityY, 0));
        engine.setSolverIterations(cfg.solverIterations);
        engine.setAerodynamics(relativeAirVelocity(cfg),
                cfg.airDrag, cfg.turbulence);
        List<BedrockModelData.Bone> allBones = boneMapper.getAllBones();
        poseEvaluator = compiledPoseEvaluatorEnabled
                ? BonePoseCalculator.compile(allBones) : null;
        long poseStarted = profiler.start(BakeProfiler.Stage.OUTER_REFERENCE_POSE);
        currentReferencePoses = calculatePoses(sourceAnimation, 0);
        profiler.stop(BakeProfiler.Stage.OUTER_REFERENCE_POSE, poseStarted);
        if (cfg.simulationMode == BoneMapper.SimulationMode.RIGID_BODY) {
            rigidBodySession = RigidBodyBakeSession.create(
                    boneMapper, sourceAnimation, currentReferencePoses, profiler);
            rigidBodySession.setPoseSampler(
                    time -> calculatePoses(sourceAnimation, time));
            rigidBodyCollisionAuditor = new RigidBodyCollisionAuditor(
                    allBones, rigidBodySession.getPhysicsBodyDefinitions(),
                    boneMapper.getExpandedCollisionBones(), cfg.rigidBodyUnitScale,
                    cfg.rigidBodyMaximumSafePenetration,
                    cfg.enableGroundCollision);
            if (activeTransition != null) {
                if (transitionPreRollSteps == 0) beginTransition(null, 0);
            } else if (hasPeriodicLoop()) {
                loopController = new LoopBakeController(loopBakeConfig,
                        new RigidBodyPeriodicStateAdapter(rigidBodySession));
                loopCycleFrames.add(createRigidBodyFrame(
                        currentReferencePoses, 0));
            } else {
                recordRigidBodyFrame(currentReferencePoses);
            }
            initialized = true;
            profiler.stop(BakeProfiler.Stage.INITIALIZE, initializeStarted);
            return;
        }
        collisionParticles = new Particle[boneMapper.getPhysicsBones().size()];
        for (String boneName : boneMapper.getPhysicsBones()) {
            boolean isFixed = boneMapper.isFixedBone(boneName);
            double mass = isFixed ? 0.0 : boneMapper.getEffectiveMass(boneName);
            if (!isFixed && !(mass > 0)) {
                throw new IllegalArgumentException(
                        "dynamic physics bone requires positive mass: " + boneName);
            }
            Particle p = new Particle(mass);
            p.setForceMultipliers(
                    boneMapper.getEffectiveGravityScale(boneName),
                    boneMapper.getEffectiveWindInfluence(boneName),
                    boneMapper.getEffectiveTurbulenceInfluence(boneName));
            BonePoseCalculator.Pose initialPose = currentReferencePoses.get(boneName);
            double[] wp = initialPose == null ? new double[]{0, 0, 0}
                    : initialPose.worldPosition;
            p.setPosition(new Vector3(wp[0], wp[1], wp[2]));
            p.setPrevPosition(new Vector3(wp[0], wp[1], wp[2]));
            engine.addParticle(p);
            int particleIndex = boneMapper.getParticleIndex(boneName);
            if (particleIndex >= 0) collisionParticles[particleIndex] = p;
        }

        List<BoneMapper.ConstraintDef> defs = boneMapper.generateChainConstraints();
        for (BoneMapper.ConstraintDef def : defs) {
            int idxA = boneMapper.getParticleIndex(def.boneA);
            int idxB = boneMapper.getParticleIndex(def.boneB);
            if (idxA >= 0 && idxB >= 0) {
                if (def.restLength == 0) {
                    engine.addConstraint(new WeldConstraint(
                            idxA, idxB, def.compliance, def.dampingCompliance));
                } else {
                    engine.addConstraint(new DistanceConstraint(
                            idxA, idxB, def.restLength, def.compliance,
                            def.dampingCompliance));
                }
            }
        }

        List<BoneMapper.CrossSpringDef> crossSpringDefs =
                boneMapper.generateCrossSpringConstraints();
        for (BoneMapper.CrossSpringDef def : crossSpringDefs) {
            int idxA = boneMapper.getParticleIndex(def.boneA);
            int idxC = boneMapper.getParticleIndex(def.boneC);
            if (idxA >= 0 && idxC >= 0) {
                engine.addConstraint(new CrossSpringConstraint(idxA, idxC,
                        def.minDistance, def.maxDistance, def.compliance,
                        def.fallbackX, def.fallbackY, def.fallbackZ));
            }
        }

        animationTargets = new TargetConstraint[boneMapper.getPhysicsBones().size()];
        for (String boneName : boneMapper.getPhysicsBones()) {
            int particleIndex = boneMapper.getParticleIndex(boneName);
            double pullCompliance = boneMapper.getEffectiveAnimPullCompliance(boneName);
            if (particleIndex < 0 || boneMapper.isFixedBone(boneName)
                    || pullCompliance <= 0) continue;
            TargetConstraint target = new TargetConstraint(particleIndex, pullCompliance);
            BonePoseCalculator.Pose pose = currentReferencePoses.get(boneName);
            if (pose != null) {
                target.setTarget(pose.worldPosition[0], pose.worldPosition[1],
                        pose.worldPosition[2]);
            }
            animationTargets[particleIndex] = target;
            engine.addConstraint(target);
        }

        Set<String> collisionBones = boneMapper.getExpandedCollisionBones();
        if (!collisionBones.isEmpty()) {
            validateCollisionAnimationScale(allBones, collisionBones, sourceAnimation);
            if (activeTransition != null) {
                validateCollisionAnimationScale(allBones, collisionBones,
                        activeTransition.targetAnimation());
            }
            bodyColliderCache = new BodyColliderCache(allBones, collisionBones);
            bodyColliderCache.initialize(currentReferencePoses);
            if (!bodyColliderCache.getColliders().isEmpty()) {
                int[] particleIndices = new int[collisionParticles.length];
                for (int i = 0; i < particleIndices.length; i++) particleIndices[i] = i;
                collisionConstraint = new VertexFaceCollisionConstraint(
                        particleIndices, collisionParticles.length, bodyColliderCache,
                        cfg.collisionSkin, cfg.xpbdCollisionRestitution);
                // 碰撞约束刻意注册在每个目标约束之后。
                engine.addConstraint(collisionConstraint);
                collisionConstraint.projectInitial(collisionParticles);
            }
        }

        if (cfg.enableGroundCollision) {
            int[] particleIndices = new int[collisionParticles.length];
            for (int i = 0; i < particleIndices.length; i++) particleIndices[i] = i;
            groundCollisionConstraint = new GroundCollisionConstraint(
                    particleIndices, collisionParticles.length, 0,
                    cfg.collisionSkin, cfg.xpbdCollisionRestitution);
            engine.addConstraint(groundCollisionConstraint);
            groundCollisionConstraint.projectInitial(collisionParticles);
        }

        if (activeTransition != null) {
            if (transitionPreRollSteps == 0) beginTransition(null, 0);
        } else if (hasPeriodicLoop()) {
            loopController = new LoopBakeController(loopBakeConfig,
                    new XpbdPeriodicStateAdapter(
                            () -> createXpbdFrame(currentReferencePoses,
                                    sourceAnimation.animationLength),
                            () -> 0.0, this::xpbdAnomalyCount));
            loopCycleFrames.add(createXpbdFrame(currentReferencePoses, 0));
        } else {
            recordFrame(currentReferencePoses);
        }
        initialized = true;
        profiler.stop(BakeProfiler.Stage.INITIALIZE, initializeStarted);
    }

    private void configureTransition() {
        activeTransition = requestedTransition;
        if (activeTransition == null && transitionAnimation != null
                && transitionAnimation != sourceAnimation
                && sourceAnimation != null
                && boneMapper.getConfig().transitionDuration > 0) {
            activeTransition = TransitionBakeRequest.endingAtClipBoundary(
                    sourceAnimation, transitionAnimation,
                    boneMapper.getConfig().transitionDuration);
        }
        transitionController = null;
        transitionStarted = false;
        transitionPreRollSteps = 0;
        transitionSteps = 0;
        transitionPreRollDt = dt;
        transitionDt = dt;
        if (activeTransition == null) return;
        if (activeTransition.sourceAnimation() != sourceAnimation) {
            throw new IllegalArgumentException(
                    "transition source must be the baker source animation");
        }
        transitionPreRollSteps = activeTransition.sourceExitTime() > 0
                ? Math.max(1, (int) Math.ceil(
                activeTransition.sourceExitTime() / dt)) : 0;
        transitionPreRollDt = transitionPreRollSteps > 0
                ? activeTransition.sourceExitTime() / transitionPreRollSteps : dt;
        transitionSteps = Math.max(1, (int) Math.ceil(
                activeTransition.transitionDuration() / dt));
        transitionDt = activeTransition.transitionDuration() / transitionSteps;
        outputFrameInterval = transitionDt;
    }

    private void configureLoopTiming() {
        cycleDt = dt;
        loopBakeConfig = null;
        if (!hasPeriodicLoop()) return;
        int cycleSteps = getCycleSteps();
        cycleDt = sourceAnimation.animationLength / cycleSteps;
        loopBakeConfig = LoopBakeConfig.from(boneMapper.getConfig());
    }

    private int calculateTotalSteps() {
        if (activeTransition != null) {
            long required = (long) transitionPreRollSteps + transitionSteps;
            if (required > MAX_BAKE_STEPS) {
                throw new IllegalArgumentException("transition requires " + required
                        + " simulation steps; maximum is " + MAX_BAKE_STEPS);
            }
            return (int) required;
        }
        if (sourceAnimation == null) return 300;
        double length = sourceAnimation.animationLength;
        if (!Double.isFinite(length) || length < 0) {
            throw new IllegalArgumentException(
                    "animation length must be a finite non-negative number");
        }
        double baseSteps = Math.ceil(length / dt);
        double requiredSteps = hasPeriodicLoop()
                ? baseSteps * loopBakeConfig.maximumWarmupCycles() : baseSteps;
        if (!Double.isFinite(requiredSteps) || requiredSteps > MAX_BAKE_STEPS) {
            throw new IllegalArgumentException("animation requires "
                    + String.format(Locale.US, "%.0f", requiredSteps)
                    + " simulation steps; maximum is " + MAX_BAKE_STEPS);
        }
        return (int) requiredSteps;
    }

    public static Vector3 windVector(double speed, double directionDegrees,
                                     double elevationDegrees) {
        double safeSpeed = Double.isFinite(speed) ? Math.max(0, speed) : 0;
        double azimuth = Math.toRadians(Double.isFinite(directionDegrees)
                ? directionDegrees : 0);
        double elevation = Math.toRadians(Double.isFinite(elevationDegrees)
                ? Math.max(-90, Math.min(90, elevationDegrees)) : 0);
        double horizontal = Math.cos(elevation) * safeSpeed;
        return new Vector3(
                Math.cos(azimuth) * horizontal,
                Math.sin(elevation) * safeSpeed,
                Math.sin(azimuth) * horizontal
        );
    }

    /** 在运动角色/模型参考系中表示的环境空气速度。 */
    public static Vector3 relativeAirVelocity(BoneMapper.PhysicsGroupConfig cfg) {
        Vector3 environment = environmentWindVelocity(cfg);
        Vector3 movement = windVector(cfg.movementSpeed, cfg.movementDirectionDegrees,
                cfg.movementElevationDegrees);
        return environment.sub(movement);
    }

    public static Vector3 environmentWindVelocity(BoneMapper.PhysicsGroupConfig cfg) {
        if (cfg.useWindComponents) {
            return new Vector3(finiteOrZero(cfg.windX), finiteOrZero(cfg.windY),
                    finiteOrZero(cfg.windZ));
        }
        return windVector(cfg.windSpeed, cfg.windDirectionDegrees,
                cfg.windElevationDegrees);
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0;
    }

    public boolean isLooping() {
        return getOutputLoopBehavior() == BedrockAnimationData.Animation.LoopBehavior.LOOP;
    }

    private boolean hasPeriodicLoop() {
        return activeTransition == null && sourceAnimation != null && isLooping()
                && Double.isFinite(sourceAnimation.animationLength)
                && sourceAnimation.animationLength > 0;
    }

    public BedrockAnimationData.Animation.LoopBehavior getOutputLoopBehavior() {
        if (activeTransition != null) {
            return BedrockAnimationData.Animation.LoopBehavior.ONCE;
        }
        BoneMapper.LoopMode mode = boneMapper.getConfig().loopMode;
        if (mode == BoneMapper.LoopMode.FORCE_LOOP) {
            return sourceAnimation == null
                    ? BedrockAnimationData.Animation.LoopBehavior.ONCE
                    : BedrockAnimationData.Animation.LoopBehavior.LOOP;
        }
        if (mode == BoneMapper.LoopMode.FORCE_ONCE || sourceAnimation == null) {
            return BedrockAnimationData.Animation.LoopBehavior.ONCE;
        }
        if (sourceAnimation.loop) {
            return BedrockAnimationData.Animation.LoopBehavior.LOOP;
        }
        return sourceAnimation.loopBehavior;
    }

    public void step() {
        if (currentStep >= totalSteps) return;
        if (activeTransition != null) {
            stepTransition();
            return;
        }

        // 本步将状态从 currentStep 推进到下一步，因此固定目标使用目标时刻。
        // 使用 currentStep 会重复第 0 帧，并使全部烘焙速度滞后一个采样。
        double time = (currentStep + 1) * dt;
        double stepDt = dt;
        boolean historyContinuous = true;
        boolean cycleBoundary = false;
        if (sourceAnimation != null && sourceAnimation.animationLength > 0) {
            if (hasPeriodicLoop()) {
                int cycleSteps = getCycleSteps();
                int phaseStep = (currentStep + 1) % cycleSteps;
                cycleBoundary = phaseStep == 0;
                time = phaseStep == 0 ? 0
                        : Math.min(phaseStep * cycleDt,
                        sourceAnimation.animationLength);
                stepDt = cycleDt;
                historyContinuous = !cycleBoundary;
            } else {
                stepDt = Math.min(dt,
                        sourceAnimation.animationLength - currentSampleTime);
                time = currentSampleTime + stepDt;
            }
        }
        if (!Double.isFinite(stepDt) || !(stepDt > 0)) {
            throw new IllegalStateException("bake step duration must be positive and finite");
        }

        long poseStarted = profiler.start(BakeProfiler.Stage.OUTER_REFERENCE_POSE);
        currentReferencePoses = calculatePoses(sourceAnimation, time);
        profiler.stop(BakeProfiler.Stage.OUTER_REFERENCE_POSE, poseStarted);
        if (rigidBodySession != null) {
            long advanceStarted = profiler.start(BakeProfiler.Stage.RIGID_BODY_ADVANCE);
            rigidBodySession.advance(currentSampleTime, time, stepDt,
                    historyContinuous || hasPeriodicLoop(), hasPeriodicLoop()
                    ? sourceAnimation.animationLength : 0,
                    endpointPoseReuseEnabled ? currentReferencePoses : null);
            profiler.stop(BakeProfiler.Stage.RIGID_BODY_ADVANCE, advanceStarted);
            currentStep++;
            currentSampleTime = time;
            recordAdvancedState(cycleBoundary);
            return;
        }
        updateFixedBones(currentReferencePoses, stepDt, historyContinuous);
        updateAnimationTargets(currentReferencePoses);
        if (bodyColliderCache != null) {
            bodyColliderCache.advance(currentReferencePoses,
                    historyContinuous || hasPeriodicLoop());
        }

        engine.step(stepDt, time, hasPeriodicLoop()
                ? sourceAnimation.animationLength : 0);
        if (collisionConstraint != null) {
            collisionConstraint.postSolveVelocity(collisionParticles);
        }
        if (groundCollisionConstraint != null) {
            groundCollisionConstraint.postSolveVelocity(collisionParticles);
        }

        currentStep++;
        currentSampleTime = time;

        recordAdvancedState(cycleBoundary);
    }

    private void stepTransition() {
        if (!transitionStarted) {
            BakedFrame previousPhysicalState = rigidBodySession != null
                    ? createRigidBodyFrame(currentReferencePoses, currentSampleTime)
                    : createXpbdFrame(currentReferencePoses, currentSampleTime);
            double previousSampleTime = currentSampleTime;
            int next = currentStep + 1;
            double time = Math.min(activeTransition.sourceExitTime(),
                    next * transitionPreRollDt);
            long poseStarted = profiler.start(BakeProfiler.Stage.OUTER_REFERENCE_POSE);
            currentReferencePoses = calculatePoses(sourceAnimation, time);
            profiler.stop(BakeProfiler.Stage.OUTER_REFERENCE_POSE, poseStarted);
            advanceSolvers(currentSampleTime, time, transitionPreRollDt,
                    true, sourceAnimation.loop ? sourceAnimation.animationLength : 0);
            currentStep++;
            currentSampleTime = time;
            if (currentStep >= transitionPreRollSteps) {
                beginTransition(previousPhysicalState,
                        Math.max(0, time - previousSampleTime));
            }
            return;
        }

        int transitionStep = currentStep - transitionPreRollSteps + 1;
        double time = Math.min(activeTransition.transitionDuration(),
                transitionStep * transitionDt);
        long transitionStartedNanos = profiler.start(
                BakeProfiler.Stage.TRANSITION_SAMPLE);
        currentReferencePoses = transitionController.sample(time);
        profiler.stop(BakeProfiler.Stage.TRANSITION_SAMPLE, transitionStartedNanos);
        double forcingPeriod = activeTransition.targetAnimation().loop
                ? activeTransition.targetAnimation().animationLength : 0;
        advanceSolvers(currentSampleTime, time, transitionDt,
                true, forcingPeriod);
        currentStep++;
        currentSampleTime = time;
        frames.add(rigidBodySession != null
                ? createRigidBodyFrame(currentReferencePoses, time)
                : createXpbdFrame(currentReferencePoses, time));
    }

    private void advanceSolvers(double startTime, double endTime, double stepDt,
                                boolean continuousHistory, double forcingPeriod) {
        if (rigidBodySession != null) {
            long advanceStarted = profiler.start(BakeProfiler.Stage.RIGID_BODY_ADVANCE);
            rigidBodySession.advance(startTime, endTime, stepDt,
                    continuousHistory, forcingPeriod,
                    endpointPoseReuseEnabled ? currentReferencePoses : null);
            profiler.stop(BakeProfiler.Stage.RIGID_BODY_ADVANCE, advanceStarted);
            return;
        }
        updateFixedBones(currentReferencePoses, stepDt, continuousHistory);
        updateAnimationTargets(currentReferencePoses);
        if (bodyColliderCache != null) {
            bodyColliderCache.advance(currentReferencePoses, continuousHistory);
        }
        engine.step(stepDt, endTime, forcingPeriod);
        if (collisionConstraint != null) {
            collisionConstraint.postSolveVelocity(collisionParticles);
        }
        if (groundCollisionConstraint != null) {
            groundCollisionConstraint.postSolveVelocity(collisionParticles);
        }
    }

    private void beginTransition(BakedFrame previousPhysicalState,
                                 double physicalFrameSpan) {
        BakedFrame physicalState = rigidBodySession != null
                ? createRigidBodyFrame(currentReferencePoses, 0)
                : createXpbdFrame(currentReferencePoses, 0);
        transitionController = new TransitionBakeController(
                activeTransition, boneMapper.getAllBones(), physicalState,
                previousPhysicalState, physicalFrameSpan);
        if (rigidBodySession != null) {
            rigidBodySession.setPoseSampler(transitionController::sample);
        }
        transitionStarted = true;
        currentSampleTime = 0;
        currentReferencePoses = transitionController.sample(0);
        frames.clear();
        frames.add(rigidBodySession != null
                ? createRigidBodyFrame(currentReferencePoses, 0)
                : createXpbdFrame(currentReferencePoses, 0));
    }

    private void recordAdvancedState(boolean cycleBoundary) {
        if (!hasPeriodicLoop()) {
            if (rigidBodySession != null) recordRigidBodyFrame(currentReferencePoses);
            else recordFrame(currentReferencePoses);
            return;
        }
        double frameTime = cycleBoundary ? sourceAnimation.animationLength
                : ((currentStep % getCycleSteps()) * cycleDt);
        BakedFrame frame = rigidBodySession != null
                ? createRigidBodyFrame(currentReferencePoses, frameTime)
                : createXpbdFrame(currentReferencePoses, frameTime);
        loopCycleFrames.add(frame);
        if (!cycleBoundary) return;

        long loopStarted = profiler.start(BakeProfiler.Stage.LOOP_CONTROLLER);
        LoopBakeController.Outcome outcome = loopController.completeCycle();
        profiler.stop(BakeProfiler.Stage.LOOP_CONTROLLER, loopStarted);
        completedLoopCycles = outcome.completedCycles();
        profiler.setCounter(BakeProfiler.Counter.WARMUP_CYCLE_COUNT,
                completedLoopCycles);
        loopErrorReport = outcome.report();
        loopConverged = outcome.converged();
        if (outcome.report() != null
                && outcome.completedCycles() >= loopBakeConfig.minimumWarmupCycles()) {
            profiler.addCounter(BakeProfiler.Counter.LOOP_CANDIDATE_COUNT, 1);
            LoopCycleCandidate candidate = new LoopCycleCandidate(
                    outcome.completedCycles(), outcome.normalizedScore(), outcome.report());
            if (candidate.isBetterThan(bestLoopCycleCandidate)) {
                bestLoopCycleCandidate = candidate;
                bestLoopCycleFrames.clear();
                bestLoopCycleFrames.addAll(copyFrames(loopCycleFrames));
            }
        }
        frames.clear();
        frames.addAll(loopCycleFrames);
        if (outcome.finished()) {
            if (!bestLoopCycleFrames.isEmpty()) {
                frames.clear();
                frames.addAll(copyFrames(bestLoopCycleFrames));
                loopErrorReport = bestLoopCycleCandidate.report();
            }
            loopFallbackUsed = outcome.useFallback();
            totalSteps = currentStep;
            return;
        }
        BakedFrame boundary = copyFrameAtTime(frame, 0);
        loopCycleFrames.clear();
        loopCycleFrames.add(boundary);
    }

    private int getCycleSteps() {
        if (sourceAnimation == null || sourceAnimation.animationLength <= 0) return 1;
        return Math.max(1, (int) Math.ceil(sourceAnimation.animationLength / dt));
    }

    private Map<String, BonePoseCalculator.Pose> calculatePoses(
            BedrockAnimationData.Animation animation, double time) {
        return compiledPoseEvaluatorEnabled
                ? poseEvaluator.calculate(animation, time)
                : BonePoseCalculator.calculate(
                boneMapper.getAllBones(), animation, time);
    }

    private Map<String, BonePoseCalculator.Pose> calculatePoses(
            BedrockAnimationData.Animation animation, double time,
            Map<String, double[]> positionOverrides,
            Map<String, double[]> rotationOverrides) {
        return compiledPoseEvaluatorEnabled
                ? poseEvaluator.calculate(animation, time,
                        positionOverrides, rotationOverrides)
                : BonePoseCalculator.calculate(boneMapper.getAllBones(), animation,
                        time, positionOverrides, rotationOverrides);
    }

    private void rebuildStructureCaches() {
        bonesByName.clear();
        physicsChildrenByBone.clear();
        orderedPhysicsBoneCache.clear();
        for (BedrockModelData.Bone bone : boneMapper.getAllBones()) {
            if (bone != null && bone.name != null) bonesByName.put(bone.name, bone);
        }
        orderedPhysicsBoneCache.addAll(boneMapper.getPhysicsBones());
        orderedPhysicsBoneCache.sort(Comparator.comparingInt(this::cachedHierarchyDepth));
        for (String name : orderedPhysicsBoneCache) {
            BedrockModelData.Bone bone = bonesByName.get(name);
            if (bone != null && bone.parent != null
                    && boneMapper.isPhysicsBone(bone.parent)) {
                physicsChildrenByBone.computeIfAbsent(bone.parent,
                        ignored -> new ArrayList<>()).add(name);
            }
        }
    }

    private int cachedHierarchyDepth(String boneName) {
        int depth = 0;
        Set<String> visited = new HashSet<>();
        BedrockModelData.Bone bone = bonesByName.get(boneName);
        while (bone != null && bone.parent != null && visited.add(bone.name)) {
            depth++;
            bone = bonesByName.get(bone.parent);
        }
        return depth;
    }

    private void updateFixedBones(Map<String, BonePoseCalculator.Pose> poses,
                                  double stepDt, boolean continuousHistory) {
        Map<String, BonePoseCalculator.Pose> previousPhase = null;
        Map<String, BonePoseCalculator.Pose> nextPhase = null;
        if (!continuousHistory && hasPeriodicLoop()) {
            double length = sourceAnimation.animationLength;
            previousPhase = calculatePoses(
                    sourceAnimation, Math.max(0, length - cycleDt));
            nextPhase = calculatePoses(
                    sourceAnimation, Math.min(length, cycleDt));
        }
        for (String boneName : boneMapper.getPhysicsBones()) {
            if (!boneMapper.isFixedBone(boneName)) continue;
            int pIdx = boneMapper.getParticleIndex(boneName);
            if (pIdx < 0) continue;
            Particle p = engine.getParticle(pIdx);
            if (!p.isFixed()) continue;

            BonePoseCalculator.Pose pose = poses.get(boneName);
            if (pose == null) continue;
            double nx = pose.worldPosition[0];
            double ny = pose.worldPosition[1];
            double nz = pose.worldPosition[2];

            if (continuousHistory) {
                p.setKinematicPosition(nx, ny, nz, stepDt);
            } else if (previousPhase != null && nextPhase != null
                    && previousPhase.get(boneName) != null
                    && nextPhase.get(boneName) != null) {
                double[] before = previousPhase.get(boneName).worldPosition;
                double[] after = nextPhase.get(boneName).worldPosition;
                double inverseSpan = 1.0 / (2.0 * cycleDt);
                p.synchronizeKinematicPosition(nx, ny, nz,
                        (after[0] - before[0]) * inverseSpan,
                        (after[1] - before[1]) * inverseSpan,
                        (after[2] - before[2]) * inverseSpan, stepDt);
            } else {
                p.synchronizeKinematicPosition(nx, ny, nz);
            }
        }
    }


    private void updateAnimationTargets(Map<String, BonePoseCalculator.Pose> poses) {
        for (String boneName : boneMapper.getPhysicsBones()) {
            int pIdx = boneMapper.getParticleIndex(boneName);
            if (pIdx < 0 || pIdx >= animationTargets.length) continue;
            TargetConstraint target = animationTargets[pIdx];
            if (target == null) continue;
            BonePoseCalculator.Pose pose = poses.get(boneName);
            if (pose == null) continue;
            target.setTarget(pose.worldPosition[0], pose.worldPosition[1],
                    pose.worldPosition[2]);
        }
    }

    public void runToEnd() {
        long started = profiler.start(BakeProfiler.Stage.TOTAL_BAKE);
        while (currentStep < totalSteps) {
            step();
        }
        finalizeFrames();
        profiler.stop(BakeProfiler.Stage.TOTAL_BAKE, started);
    }

    public void runSteps(int n) {
        for (int i = 0; i < n && currentStep < totalSteps; i++) {
            step();
        }
    }

    public void finalizeFrames() {
        if (framesFinalized) return;
        if (activeTransition == null) {
            long resampleStarted = profiler.start(BakeProfiler.Stage.RESAMPLE);
            resampleFramesToSourceTimeline();
            profiler.stop(BakeProfiler.Stage.RESAMPLE, resampleStarted);
            long blendStarted = profiler.start(BakeProfiler.Stage.BLEND);
            blendOrdinaryFramesToReferenceEdges();
            profiler.stop(BakeProfiler.Stage.BLEND, blendStarted);
        }
        if (hasPeriodicLoop()) {
            long loopStarted = profiler.start(BakeProfiler.Stage.LOOP_FINALIZATION);
            finalizeLoopSeam();
            profiler.stop(BakeProfiler.Stage.LOOP_FINALIZATION, loopStarted);
        }
        if (activeTransition != null) {
            long blendStarted = profiler.start(BakeProfiler.Stage.BLEND);
            blendTransitionFramesToReference();
            profiler.stop(BakeProfiler.Stage.BLEND, blendStarted);
        }
        long unwrapStarted = profiler.start(BakeProfiler.Stage.UNWRAP);
        unwrapFinalRotations();
        profiler.stop(BakeProfiler.Stage.UNWRAP, unwrapStarted);
        long auditStarted = profiler.start(BakeProfiler.Stage.FINAL_COLLISION_AUDIT);
        rebuildFinalWorldPositionsAndAudit();
        profiler.stop(BakeProfiler.Stage.FINAL_COLLISION_AUDIT, auditStarted);
        framesFinalized = true;
        profiler.setCounter(BakeProfiler.Counter.OUTPUT_FRAME_COUNT, frames.size());
    }

    /**
     * Smoothly hands an ordinary non-loop physical bake back to its source
     * animation at both clip edges. Explicit A-to-B transitions and periodic
     * loops have separate boundary handling.
     */
    private void blendOrdinaryFramesToReferenceEdges() {
        double requested = boneMapper.getConfig().transitionDuration;
        if (sourceAnimation == null || transitionAnimation != sourceAnimation
                || hasPeriodicLoop() || !Double.isFinite(requested)
                || requested <= 0 || frames.size() < 2) {
            return;
        }
        double length = frames.get(frames.size() - 1).time;
        if (!(length > 0)) return;
        double duration = Math.min(requested, length * 0.5);

        for (BakedFrame frame : frames) {
            double edgeDistance = Math.min(frame.time, length - frame.time);
            if (edgeDistance >= duration) continue;
            double referenceWeight = 1 - smootherStep(edgeDistance / duration);
            double referenceTime = frame.time > length * 0.5
                    ? Math.max(0, sourceAnimation.animationLength) : 0;
            Map<String, BonePoseCalculator.Pose> referencePoses =
                    calculatePoses(sourceAnimation, referenceTime);

            for (BoneState state : frame.boneStates) {
                BedrockModelData.Bone bone = bonesByName.get(state.boneName);
                BonePoseCalculator.Pose reference = referencePoses.get(state.boneName);
                if (bone == null || reference == null) continue;

                if (state.position != null) {
                    for (int axis = 0; axis < 3; axis++) {
                        state.position[axis] += referenceWeight
                                * (reference.animationPosition[axis]
                                - state.position[axis]);
                    }
                }
                if (state.rotation != null) {
                    double[] physicalTotal = new double[]{
                            bone.rotation[0] + state.rotation[0],
                            bone.rotation[1] + state.rotation[1],
                            bone.rotation[2] + state.rotation[2]
                    };
                    double[] fromQ = RotationUtil.quaternionFromBedrockEuler(
                            physicalTotal[0], physicalTotal[1], physicalTotal[2]);
                    double[] toQ = RotationUtil.quaternionFromBedrockEuler(
                            reference.totalLocalEuler[0],
                            reference.totalLocalEuler[1],
                            reference.totalLocalEuler[2]);
                    double[] delta = RotationUtil.rotationVectorFromQuaternion(
                            RotationUtil.quaternionMultiply(
                                    toQ, RotationUtil.quaternionInverse(fromQ)));
                    for (int axis = 0; axis < 3; axis++) {
                        delta[axis] *= referenceWeight;
                    }
                    double[] blendedQ = RotationUtil.quaternionMultiply(
                            RotationUtil.quaternionFromRotationVector(delta), fromQ);
                    double[] blendedTotal = RotationUtil.unwrapEuler(
                            physicalTotal,
                            RotationUtil.bedrockEulerFromQuaternion(blendedQ));
                    for (int axis = 0; axis < 3; axis++) {
                        state.rotation[axis] = blendedTotal[axis] - bone.rotation[axis];
                    }
                }
                if (state.linearVelocity != null) {
                    for (int axis = 0; axis < 3; axis++) {
                        state.linearVelocity[axis] *= 1 - referenceWeight;
                    }
                }
            }
        }
    }

    /**
     * A transition clip is a contract between A's live physical exit and B's
     * sampled pose. The solver remains authoritative in the middle, while this
     * endpoint blend guarantees that the exported last keyframe can actually
     * continue into B even when compliance, inertia, or collision made the
     * physical state lag behind its reference target.
     */
    private void blendTransitionFramesToReference() {
        if (transitionController == null || activeTransition == null
                || frames.size() < 2) return;
        double duration = activeTransition.transitionDuration();
        if (!(duration > 0)) return;

        for (BakedFrame frame : frames) {
            double weight = smootherStep(frame.time / duration);
            if (weight <= 0) continue;
            Map<String, BonePoseCalculator.Pose> referencePoses =
                    transitionController.sample(frame.time);
            for (BoneState state : frame.boneStates) {
                BedrockModelData.Bone bone = bonesByName.get(state.boneName);
                BonePoseCalculator.Pose reference = referencePoses.get(state.boneName);
                if (bone == null || reference == null) continue;

                if (state.position != null) {
                    for (int axis = 0; axis < 3; axis++) {
                        state.position[axis] += weight
                                * (reference.animationPosition[axis]
                                - state.position[axis]);
                    }
                }
                if (state.rotation != null) {
                    double[] physicalTotal = new double[]{
                            bone.rotation[0] + state.rotation[0],
                            bone.rotation[1] + state.rotation[1],
                            bone.rotation[2] + state.rotation[2]
                    };
                    double[] fromQ = RotationUtil.quaternionFromBedrockEuler(
                            physicalTotal[0], physicalTotal[1], physicalTotal[2]);
                    double[] toQ = RotationUtil.quaternionFromBedrockEuler(
                            reference.totalLocalEuler[0],
                            reference.totalLocalEuler[1],
                            reference.totalLocalEuler[2]);
                    double[] delta = RotationUtil.rotationVectorFromQuaternion(
                            RotationUtil.quaternionMultiply(
                                    toQ, RotationUtil.quaternionInverse(fromQ)));
                    for (int axis = 0; axis < 3; axis++) delta[axis] *= weight;
                    double[] blendedQ = RotationUtil.quaternionMultiply(
                            RotationUtil.quaternionFromRotationVector(delta), fromQ);
                    double[] blendedTotal = RotationUtil.unwrapEuler(
                            physicalTotal,
                            RotationUtil.bedrockEulerFromQuaternion(blendedQ));
                    for (int axis = 0; axis < 3; axis++) {
                        state.rotation[axis] = blendedTotal[axis] - bone.rotation[axis];
                    }
                }
                if (state.linearVelocity != null) {
                    for (int axis = 0; axis < 3; axis++) {
                        state.linearVelocity[axis] *= 1 - weight;
                    }
                }
            }
        }
    }

    private static double smootherStep(double value) {
        double t = Math.max(0, Math.min(1, value));
        return t * t * t * (t * (t * 6 - 15) + 10);
    }

    public boolean isFramesFinalized() {
        return framesFinalized;
    }

    public void requireSafeForExport() {
        if (!framesFinalized || currentStep < totalSteps) {
            throw new IllegalStateException("the bake must be complete and finalized before export");
        }
        if (getUnsafeFinalCollisionCount() > 0) {
            throw new IllegalStateException("final collision audit rejected the bake");
        }
        if (loopSeamCorrectionRejected) {
            throw new IllegalStateException(
                    "loop seam correction could not satisfy continuity and collision gates");
        }
    }

    private void unwrapFinalRotations() {
        Map<String, double[]> previousByBone = new HashMap<>();
        for (BakedFrame frame : frames) {
            for (BoneState state : frame.boneStates) {
                if (state.rotation == null || state.rotation.length < 3) continue;
                double[] previous = previousByBone.get(state.boneName);
                if (previous != null) {
                    double[] unwrapped = RotationUtil.unwrapEuler(previous, state.rotation);
                    System.arraycopy(unwrapped, 0, state.rotation, 0, 3);
                }
                previousByBone.put(state.boneName, state.rotation.clone());
            }
        }
    }

    private void rebuildFinalWorldPositionsAndAudit() {
        unsafeFinalCollisionCount = rigidBodySession == null
                ? 0 : rigidBodySession.getUnsafeCollisionCount();
        for (BakedFrame frame : frames) {
            Map<String, double[]> positionOverrides = new HashMap<>();
            Map<String, double[]> rotationOverrides = new HashMap<>();
            for (BoneState state : frame.boneStates) {
                if (state.position != null) {
                    positionOverrides.put(state.boneName, state.position);
                }
                if (state.rotation != null) {
                    rotationOverrides.put(state.boneName, state.rotation);
                }
            }
            Map<String, BonePoseCalculator.Pose> poses = calculatePoses(
                    activeTransition == null
                            ? sourceAnimation : activeTransition.targetAnimation(),
                    activeTransition == null ? frame.time
                            : transitionController.targetSampleTime(frame.time),
                    positionOverrides, rotationOverrides);
            if (bodyColliderCache != null) bodyColliderCache.setAuditPose(poses);
            for (BoneState state : frame.boneStates) {
                BonePoseCalculator.Pose pose = poses.get(state.boneName);
                if (pose == null || state.worldPosition == null
                        || state.worldPosition.length < 3) continue;
                state.worldPosition[0] = pose.worldPosition[0];
                state.worldPosition[1] = pose.worldPosition[1];
                state.worldPosition[2] = pose.worldPosition[2];
                if (collisionConstraint != null
                        && !boneMapper.isFixedBone(state.boneName)
                        && bodyColliderCache.containsCurrent(
                        pose.worldPosition[0], pose.worldPosition[1], pose.worldPosition[2],
                        collisionConstraint.getSkin())) {
                    unsafeFinalCollisionCount++;
                }
                if (groundCollisionConstraint != null
                        && !boneMapper.isFixedBone(state.boneName)
                        && pose.worldPosition[1]
                        < groundCollisionConstraint.getMinimumY() - 1e-9) {
                    unsafeFinalCollisionCount++;
                }
            }
            if (rigidBodyCollisionAuditor != null) {
                RigidBodyCollisionAuditor.AuditResult audit =
                        rigidBodyCollisionAuditor.audit(poses);
                maximumFinalRigidBodyPenetration = Math.max(
                        maximumFinalRigidBodyPenetration, audit.maximumPenetration());
                if (audit.unsafe()) unsafeFinalCollisionCount++;
            }
        }
    }

    private void finalizeLoopSeam() {
        CandidateAudit baselineAudit = auditLoopCandidate(frames);
        boolean solverSafe = rigidBodySession == null
                || rigidBodySession.getUnsafeCollisionCount() == 0;
        loopSeamReport = measureLoopSeam(frames, false, 0,
                solverSafe && baselineAudit.safe(), baselineAudit.maximumPenetration());
        loopFallbackUsed = false;
        bestLoopSeamCandidateReport = null;
        if (loopSeamReport.passes(boneMapper.getConfig())) {
            loopSeamCorrectionRejected = false;
            return;
        }
        if (!loopBakeConfig.seamFallbackEnabled()) {
            loopSeamCorrectionRejected = true;
            return;
        }

        double baselineMaximum = baselineAudit.maximumPenetration();
        for (double ratio : correctionWindowRatios(
                boneMapper.getConfig().loopSeamWindowRatio)) {
            LoopSeamCorrector.Result correction = LoopSeamCorrector.correctHierarchyCopy(
                    frames, boneMapper.getAllBones(), sourceAnimation,
                    boneMapper.getPhysicsBones(), fixedBones(),
                    boneMapper.getConfig().loopSeamStrategy, ratio,
                    boneMapper.getConfig().loopSeamMatchAcceleration);
            CandidateAudit audit = auditLoopCandidate(correction.frames());
            boolean penetrationPreserved = audit.maximumPenetration()
                    <= baselineMaximum + 0.01 + 1e-12;
            boolean safe = solverSafe && audit.safe() && penetrationPreserved;
            LoopSeamReport candidateReport = measureLoopSeam(correction.frames(),
                    true, correction.windowRatio(), safe,
                    audit.maximumPenetration());
            if (bestLoopSeamCandidateReport == null
                    || candidateReport.qualityScore(boneMapper.getConfig())
                    < bestLoopSeamCandidateReport.qualityScore(
                    boneMapper.getConfig())) {
                bestLoopSeamCandidateReport = candidateReport;
            }
            if (!candidateReport.passes(boneMapper.getConfig())) continue;
            frames.clear();
            frames.addAll(copyFrames(correction.frames()));
            loopSeamReport = candidateReport;
            loopFallbackUsed = true;
            loopSeamCorrectionRejected = false;
            return;
        }
        loopSeamCorrectionRejected = true;
    }

    private LoopSeamReport measureLoopSeam(List<BakedFrame> candidateFrames,
                                           boolean corrected, double windowRatio,
                                           boolean collisionSafe,
                                           double maximumPenetration) {
        Set<String> fixedBones = fixedBones();
        return LoopSeamReport.measure(candidateFrames, boneMapper.getAllBones(),
                sourceAnimation, boneMapper.getPhysicsBones(), fixedBones,
                corrected, windowRatio, collisionSafe, maximumPenetration);
    }

    private Set<String> fixedBones() {
        Set<String> fixedBones = new LinkedHashSet<>();
        for (String boneName : boneMapper.getPhysicsBones()) {
            if (boneMapper.isFixedBone(boneName)) fixedBones.add(boneName);
        }
        return fixedBones;
    }

    private CandidateAudit auditLoopCandidate(List<BakedFrame> candidateFrames) {
        if (rigidBodyCollisionAuditor == null) {
            if (groundCollisionConstraint == null) return new CandidateAudit(true, 0);
            boolean safe = true;
            double maximumPenetration = 0;
            for (BakedFrame frame : candidateFrames) {
                Map<String, BonePoseCalculator.Pose> poses = calculateFramePoses(frame);
                for (String boneName : boneMapper.getPhysicsBones()) {
                    if (boneMapper.isFixedBone(boneName)) continue;
                    BonePoseCalculator.Pose pose = poses.get(boneName);
                    if (pose == null) continue;
                    double penetration = groundCollisionConstraint.getMinimumY()
                            - pose.worldPosition[1];
                    if (penetration > 1e-9) {
                        safe = false;
                        maximumPenetration = Math.max(maximumPenetration, penetration);
                    }
                }
            }
            return new CandidateAudit(safe, maximumPenetration);
        }
        boolean safe = true;
        double maximumPenetration = 0;
        for (BakedFrame frame : candidateFrames) {
            RigidBodyCollisionAuditor.AuditResult audit =
                    rigidBodyCollisionAuditor.audit(calculateFramePoses(frame));
            safe &= !audit.unsafe();
            maximumPenetration = Math.max(maximumPenetration,
                    audit.maximumPenetration());
        }
        return new CandidateAudit(safe, maximumPenetration);
    }

    private Map<String, BonePoseCalculator.Pose> calculateFramePoses(BakedFrame frame) {
        Map<String, double[]> positionOverrides = new HashMap<>();
        Map<String, double[]> rotationOverrides = new HashMap<>();
        for (BoneState state : frame.boneStates) {
            if (state.position != null) {
                positionOverrides.put(state.boneName, state.position);
            }
            if (state.rotation != null) {
                rotationOverrides.put(state.boneName, state.rotation);
            }
        }
        return calculatePoses(sourceAnimation, frame.time,
                positionOverrides, rotationOverrides);
    }

    private static List<Double> correctionWindowRatios(double requested) {
        LinkedHashSet<Double> ratios = new LinkedHashSet<>();
        double initial = Double.isFinite(requested)
                ? Math.max(0.05, Math.min(0.5, requested)) : 0.25;
        ratios.add(initial);
        if (initial < 0.375) ratios.add(0.375);
        if (initial < 0.5) ratios.add(0.5);
        return List.copyOf(ratios);
    }

    private record CandidateAudit(boolean safe, double maximumPenetration) {
    }

    private void validateCollisionAnimationScale(List<BedrockModelData.Bone> allBones,
                                                 Set<String> collisionBones,
                                                 BedrockAnimationData.Animation drivingAnimation) {
        if (drivingAnimation == null) return;
        Map<String, BedrockModelData.Bone> byName = new HashMap<>();
        for (BedrockModelData.Bone bone : allBones) {
            if (bone != null && bone.name != null) byName.put(bone.name, bone);
        }
        Set<String> checked = new HashSet<>();
        for (String collisionBone : collisionBones) {
            BedrockModelData.Bone bone = byName.get(collisionBone);
            while (bone != null && checked.add(bone.name)) {
                BedrockAnimationData.BoneAnimation animation =
                        drivingAnimation.bones.get(bone.name);
                if (animation != null && animation.scale != null) {
                    for (double[] value : animation.scale.keyframes.values()) {
                        if (value == null || value.length < 3
                                || Math.abs(value[0] - 1) > 1e-9
                                || Math.abs(value[1] - 1) > 1e-9
                                || Math.abs(value[2] - 1) > 1e-9) {
                            throw new IllegalArgumentException(
                                    "collision bone or ancestor uses unsupported animation scale: "
                                            + bone.name);
                        }
                    }
                }
                bone = bone.parent == null ? null : byName.get(bone.parent);
            }
        }
    }

    /**
     * Keep the solver at its stable fixed dt, then align exported frames to the
     * source animation's observable frame grid. Bedrock files do not store FPS,
     * so the coarsest common grid of at least 20 FPS is inferred from key times.
     */
    private void resampleFramesToSourceTimeline() {
        if (sourceAnimation == null || frames.size() < 3) return;
        double interval = inferSourceFrameInterval();
        if (!Double.isFinite(interval) || interval <= dt * 1.0001) return;

        double length = frames.get(frames.size() - 1).time;
        if (!Double.isFinite(length) || length <= 0) return;

        List<BakedFrame> sourceFrames = new ArrayList<>(frames);
        List<BakedFrame> snapped = new ArrayList<>();
        int wholeSteps = (int) Math.floor(length / interval + 1e-9);
        int lowerIndex = 0;
        for (int i = 0; i <= wholeSteps; i++) {
            double time = i * interval;
            while (lowerIndex + 1 < sourceFrames.size()
                    && sourceFrames.get(lowerIndex + 1).time < time - 1e-12) {
                lowerIndex++;
            }
            snapped.add(interpolateFrame(sourceFrames, lowerIndex, time));
        }
        if (snapped.isEmpty()
                || Math.abs(snapped.get(snapped.size() - 1).time - length) > 1e-9) {
            snapped.add(interpolateFrame(sourceFrames,
                    Math.max(0, sourceFrames.size() - 2), length));
        }
        frames.clear();
        frames.addAll(snapped);
        outputFrameInterval = interval;
    }

    private double inferSourceFrameInterval() {
        TreeSet<Double> keyTimes = new TreeSet<>();
        for (BedrockAnimationData.BoneAnimation bone : sourceAnimation.bones.values()) {
            if (bone == null) continue;
            collectKeyTimes(keyTimes, bone.position);
            collectKeyTimes(keyTimes, bone.rotation);
            collectKeyTimes(keyTimes, bone.scale);
        }
        if (keyTimes.size() < 3) return dt;

        // 常见 DCC/Bedrock 时间线帧率。从 20 FPS 开始，可避免源 FPS 未写入 JSON 时
        // 将稀疏片段降为明显卡顿的 1–10 FPS。
        int[] candidates = {20, 24, 25, 30, 40, 48, 50, 60, 72, 90, 120, 144, 240};
        for (int fps : candidates) {
            boolean aligned = true;
            for (double time : keyTimes) {
                double frame = time * fps;
                if (Math.abs(frame - Math.rint(frame)) > 0.015) {
                    aligned = false;
                    break;
                }
            }
            if (aligned) return 1.0 / fps;
        }
        return dt;
    }

    private static void collectKeyTimes(Set<Double> target,
                                        BedrockAnimationData.Keyframes channel) {
        if (channel == null) return;
        for (Double time : channel.keyframes.keySet()) {
            if (time != null && Double.isFinite(time) && time >= 0) target.add(time);
        }
    }

    private static BakedFrame interpolateFrame(List<BakedFrame> sourceFrames,
                                               int suggestedLowerIndex,
                                               double time) {
        int lowerIndex = Math.max(0, Math.min(suggestedLowerIndex,
                sourceFrames.size() - 1));
        while (lowerIndex + 1 < sourceFrames.size()
                && sourceFrames.get(lowerIndex + 1).time < time - 1e-12) {
            lowerIndex++;
        }
        BakedFrame lower = sourceFrames.get(lowerIndex);
        BakedFrame upper = sourceFrames.get(Math.min(lowerIndex + 1,
                sourceFrames.size() - 1));
        double span = upper.time - lower.time;
        double weight = span > 1e-12
                ? Math.max(0, Math.min(1, (time - lower.time) / span)) : 0;

        List<BoneState> states = new ArrayList<>(lower.boneStates.size());
        for (BoneState from : lower.boneStates) {
            BoneState to = upper.getBoneState(from.boneName);
            if (to == null) to = from;
            states.add(new BoneState(from.boneName,
                    lerpVector(from.position, to.position, weight, false),
                    lerpVector(from.rotation, to.rotation, weight, true),
                    lerpVector(from.linearVelocity, to.linearVelocity, weight, false),
                    lerpVector(from.worldPosition, to.worldPosition, weight, false)));
        }
        return new BakedFrame(time, states);
    }

    private static double[] lerpVector(double[] from, double[] to,
                                       double weight, boolean angles) {
        if (from == null && to == null) return new double[0];
        double[] a = from != null ? from : to;
        double[] b = to != null ? to : from;
        int length = Math.min(a.length, b.length);
        double[] result = new double[length];
        for (int i = 0; i < length; i++) {
            result[i] = angles ? lerpAngle(a[i], b[i], weight)
                    : a[i] * (1 - weight) + b[i] * weight;
        }
        return result;
    }

    private static double lerpAngle(double from, double to, double t) {
        double delta = (to - from) % 360.0;
        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;
        return from + delta * t;
    }

    /**
     * Blend the tail of the loop animation back toward the first frame to ensure seamless looping.
     * Blends the last 10% of frames with a linear ramp toward frame 0.
     */
    @SuppressWarnings({"unused", "squid:S3776", "squid:S1144"})
    private void blendLoopFrames() {
        if (frames.size() < 4) return;
        int blendCount = Math.max(2, frames.size() / 10);
        BakedFrame firstFrame = frames.get(0);
        int startIdx = frames.size() - blendCount;

        for (int i = startIdx; i < frames.size(); i++) {
            // 最后一帧必须与第 0 帧完全一致，否则 loop=true 仍会在 JSON 播放边界留下可见跳变。
            double t = (double) (i - startIdx) / (blendCount - 1);
            BakedFrame frame = frames.get(i);
            for (int b = 0; b < frame.boneStates.size(); b++) {
                BoneState current = frame.boneStates.get(b);
                BoneState target = firstFrame.getBoneState(current.boneName);
                if (target == null) continue;

                // 线性插值位置
                if (current.position != null && target.position != null) {
                    for (int j = 0; j < 3; j++) {
                        current.position[j] = current.position[j] * (1 - t) + target.position[j] * t;
                    }
                }
                // 插值旋转
                if (current.rotation != null && target.rotation != null) {
                    for (int j = 0; j < 3; j++) {
                        current.rotation[j] = lerpAngle(
                                current.rotation[j], target.rotation[j], t);
                    }
                }
                // 同时保持附属速度缓存在线性循环边界处连续。
                if (current.linearVelocity != null && target.linearVelocity != null) {
                    for (int j = 0; j < 3; j++) {
                        current.linearVelocity[j] = current.linearVelocity[j] * (1 - t)
                                + target.linearVelocity[j] * t;
                    }
                }
                if (current.worldPosition != null && target.worldPosition != null) {
                    for (int j = 0; j < 3; j++) {
                        current.worldPosition[j] = current.worldPosition[j] * (1 - t)
                                + target.worldPosition[j] * t;
                    }
                }
            }
        }
    }

    public double[] getCurrentWorldPosition(String boneName) {
        if (rigidBodySession != null) {
            return rigidBodySession.getCurrentWorldPosition(boneName);
        }
        int pIdx = boneMapper.getParticleIndex(boneName);
        if (pIdx < 0) return new double[0];
        Particle p = engine.getParticle(pIdx);
        return new double[]{p.getPosition().x, p.getPosition().y, p.getPosition().z};
    }

    public double[] getCurrentReferenceWorldPosition(String boneName) {
        BonePoseCalculator.Pose pose = currentReferencePoses.get(boneName);
        return pose == null ? null : pose.worldPosition.clone();
    }

    public int getBodyColliderCount() {
        if (rigidBodySession != null) return rigidBodySession.getCollisionBodyCount();
        return bodyColliderCache == null ? 0 : bodyColliderCache.getColliders().size();
    }

    public int getDegenerateColliderCount() {
        if (rigidBodySession != null) {
            return rigidBodySession.getSkippedDegenerateCubeCount();
        }
        return bodyColliderCache == null ? 0 : bodyColliderCache.getDegenerateCubeCount();
    }

    public int getUnsafeFinalCollisionCount() {
        return rigidBodySession == null ? unsafeFinalCollisionCount
                : Math.max(unsafeFinalCollisionCount,
                rigidBodySession.getUnsafeCollisionCount());
    }

    public VertexFaceCollisionConstraint.Diagnostics getCollisionDiagnostics() {
        return collisionConstraint == null ? null : collisionConstraint.getDiagnostics();
    }

    public boolean isRigidBodyMode() {
        return rigidBodySession != null;
    }

    public int getRigidBodyPhysicsBodyCount() {
        return rigidBodySession == null ? 0 : rigidBodySession.getPhysicsBodyCount();
    }

    public int getRigidBodySourceCubeCount() {
        return rigidBodySession == null ? 0 : rigidBodySession.getSourceCubeCount();
    }

    public int getRigidBodySkippedBoneCount() {
        return rigidBodySession == null ? 0 : rigidBodySession.getSkippedBodyBoneCount();
    }

    public long getRigidBodySweepHitCount() {
        return rigidBodySession == null ? 0 : rigidBodySession.getSweepHitCount();
    }

    public int getRigidBodyContactCount() {
        return rigidBodySession == null ? 0 : rigidBodySession.getCurrentContactCount();
    }

    public double getRigidBodyMaximumPenetration() {
        return rigidBodySession == null ? 0 : Math.max(
                rigidBodySession.getMaximumPenetration(),
                maximumFinalRigidBodyPenetration);
    }

    public LoopErrorReport getLoopErrorReport() {
        return loopErrorReport;
    }

    public LoopSeamReport getLoopSeamReport() {
        return loopSeamReport;
    }

    public LoopSeamReport getBestLoopSeamCandidateReport() {
        return bestLoopSeamCandidateReport;
    }

    public boolean isLoopSeamCorrectionRejected() {
        return loopSeamCorrectionRejected;
    }

    public boolean isLoopConverged() {
        return loopConverged;
    }

    public boolean isLoopFallbackUsed() {
        return loopFallbackUsed;
    }

    public int getCompletedLoopCycles() {
        return completedLoopCycles;
    }

    public int getSelectedLoopCycle() {
        return bestLoopCycleCandidate == null ? completedLoopCycles
                : bestLoopCycleCandidate.cycleIndex();
    }

    public double getSelectedLoopCycleScore() {
        return bestLoopCycleCandidate == null ? Double.NaN
                : bestLoopCycleCandidate.normalizedScore();
    }

    public boolean isTransitionBake() {
        return activeTransition != null;
    }

    public BedrockAnimationData.Animation getOutputReferenceAnimation() {
        return activeTransition == null || !transitionStarted
                ? sourceAnimation : activeTransition.targetAnimation();
    }

    public double getOutputReferenceTime(double outputTime) {
        return activeTransition == null || !transitionStarted
                || transitionController == null
                ? outputTime : transitionController.targetSampleTime(outputTime);
    }

    public List<RigidBodyBakeSession.SubstepSnapshot> getRigidBodySubstepSnapshots() {
        return rigidBodySession == null
                ? List.of() : rigidBodySession.getLatestSubstepSnapshots();
    }

    public int getNativeBulletVersion() {
        return rigidBodySession == null ? 0 : rigidBodySession.getNativeBulletVersion();
    }

    private void recordRigidBodyFrame(
            Map<String, BonePoseCalculator.Pose> referencePoses) {
        frames.add(createRigidBodyFrame(referencePoses, currentFrameTime()));
    }

    private BakedFrame createRigidBodyFrame(
            Map<String, BonePoseCalculator.Pose> referencePoses, double frameTime) {
        List<BoneState> boneStates = new ArrayList<>();
        for (RigidBodyBakeSession.BoneOutput output
                : rigidBodySession.captureBoneOutputs(referencePoses)) {
            boneStates.add(new BoneState(output.boneName(), output.position(),
                    output.rotation(), output.linearVelocity(),
                    output.worldPosition()));
        }
        return new BakedFrame(frameTime, boneStates);
    }

    private void recordFrame(Map<String, BonePoseCalculator.Pose> referencePoses) {
        frames.add(createXpbdFrame(referencePoses, currentFrameTime()));
    }

    private BakedFrame createXpbdFrame(
            Map<String, BonePoseCalculator.Pose> referencePoses, double frameTime) {
        List<BoneState> boneStates = new ArrayList<>();

        if (referencePoses == null) {
            referencePoses = calculatePoses(sourceAnimation, frameTime);
        }
        Map<String, double[]> desiredWorldRotations = new HashMap<>();

        for (String boneName : orderedPhysicsBoneCache) {
            int pIdx = boneMapper.getParticleIndex(boneName);
            if (pIdx < 0) continue;
            Particle p = engine.getParticle(pIdx);
            BedrockModelData.Bone bone = bonesByName.get(boneName);
            BonePoseCalculator.Pose reference = referencePoses.get(boneName);
            if (bone == null || reference == null) continue;

            String parentName = bone.parent;
            double[] parentWorldQ = new double[]{0, 0, 0, 1};
            if (parentName != null) {
                parentWorldQ = desiredWorldRotations.get(parentName);
                if (parentWorldQ == null) {
                    BonePoseCalculator.Pose parentReference = referencePoses.get(parentName);
                    parentWorldQ = parentReference == null
                            ? new double[]{0, 0, 0, 1}
                            : parentReference.worldRotation;
                }
            }

            double[] desiredWorldQ = inheritedReferenceRotation(
                    reference, referencePoses.get(parentName), parentWorldQ);
            List<double[]> referenceDirections = new ArrayList<>();
            List<double[]> simulatedDirections = new ArrayList<>();
            for (String childName : physicsChildrenByBone.getOrDefault(
                    boneName, Collections.emptyList())) {
                BonePoseCalculator.Pose childReference = referencePoses.get(childName);
                int childIndex = boneMapper.getParticleIndex(childName);
                if (childReference == null || childIndex < 0) continue;
                Particle childParticle = engine.getParticle(childIndex);
                double[] referenceDirection = subtract(
                        childReference.worldPosition, reference.worldPosition);
                double[] simulatedDirection = new double[]{
                        childParticle.getPosition().x - p.getPosition().x,
                        childParticle.getPosition().y - p.getPosition().y,
                        childParticle.getPosition().z - p.getPosition().z
                };
                if (lengthSquared(referenceDirection) > 1e-12
                        && lengthSquared(simulatedDirection) > 1e-12) {
                    referenceDirections.add(referenceDirection);
                    simulatedDirections.add(simulatedDirection);
                }
            }
            if (!referenceDirections.isEmpty()) {
                double[] delta = RotationUtil.quaternionFromDirectionPairs(
                        referenceDirections.toArray(new double[0][]),
                        simulatedDirections.toArray(new double[0][]));
                desiredWorldQ = RotationUtil.quaternionMultiply(
                        delta, reference.worldRotation);
            }
            desiredWorldRotations.put(boneName, desiredWorldQ);

            double[] localQ = RotationUtil.quaternionMultiply(
                    RotationUtil.quaternionInverse(parentWorldQ), desiredWorldQ);
            double[] totalLocalEuler = RotationUtil.bedrockEulerFromQuaternion(localQ);
            double[] rotation = new double[]{
                    totalLocalEuler[0] - bone.rotation[0],
                    totalLocalEuler[1] - bone.rotation[1],
                    totalLocalEuler[2] - bone.rotation[2]
            };
            double[] localAnimationPosition = reference.animationPosition.clone();
            double[] worldPosition = {
                    p.getPosition().x, p.getPosition().y, p.getPosition().z
            };
            double[] linearVelocity = {
                    p.getVelocity().x,
                    p.getVelocity().y,
                    p.getVelocity().z
            };
            boneStates.add(new BoneState(boneName, localAnimationPosition, rotation,
                    linearVelocity, worldPosition));
        }
        return new BakedFrame(frameTime, boneStates);
    }

    private double currentFrameTime() {
        double frameTime;
        if (sourceAnimation != null && isLooping() && sourceAnimation.animationLength > 0) {
            int cycleSteps = getCycleSteps();
            frameTime = Math.min((currentStep - cycleSteps) * cycleDt,
                    sourceAnimation.animationLength);
        } else {
            frameTime = sourceAnimation != null && sourceAnimation.animationLength > 0
                    ? Math.min(currentStep * dt, sourceAnimation.animationLength)
                    : currentStep * dt;
        }
        return frameTime;
    }

    private static BakedFrame copyFrameAtTime(BakedFrame frame, double time) {
        List<BoneState> copy = new ArrayList<>(frame.boneStates.size());
        for (BoneState state : frame.boneStates) {
            copy.add(new BoneState(state.boneName,
                    cloneOrNull(state.position), cloneOrNull(state.rotation),
                    cloneOrNull(state.linearVelocity),
                    cloneOrNull(state.worldPosition)));
        }
        return new BakedFrame(time, copy);
    }

    private static List<BakedFrame> copyFrames(List<BakedFrame> source) {
        List<BakedFrame> copy = new ArrayList<>(source.size());
        for (BakedFrame frame : source) {
            copy.add(copyFrameAtTime(frame, frame.time));
        }
        return copy;
    }

    private static double[] cloneOrNull(double[] value) {
        return value == null ? null : value.clone();
    }

    private int xpbdAnomalyCount() {
        if (collisionConstraint == null) return 0;
        long invalid = collisionConstraint.getDiagnostics().invalidValues;
        return invalid >= Integer.MAX_VALUE ? Integer.MAX_VALUE : (int) invalid;
    }

    private double[] inheritedReferenceRotation(BonePoseCalculator.Pose reference,
                                                BonePoseCalculator.Pose parentReference,
                                                double[] desiredParentWorldQ) {
        if (parentReference == null) return reference.worldRotation;
        double[] referenceLocalQ = RotationUtil.quaternionMultiply(
                RotationUtil.quaternionInverse(parentReference.worldRotation),
                reference.worldRotation);
        return RotationUtil.quaternionMultiply(desiredParentWorldQ, referenceLocalQ);
    }

    private static double[] subtract(double[] a, double[] b) {
        return new double[]{a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    private static double lengthSquared(double[] value) {
        return value[0] * value[0] + value[1] * value[1] + value[2] * value[2];
    }

    @Override
    public void close() {
        closeRigidBodySession();
        initialized = false;
    }

    /** 释放求解器资源，并清空本次烘焙的输出和诊断信息。 */
    public void reset() {
        closeRigidBodySession();
        frames.clear();
        loopCycleFrames.clear();
        bestLoopCycleFrames.clear();
        currentStep = 0;
        totalSteps = 0;
        currentSampleTime = 0;
        framesFinalized = false;
        initialized = false;
        animationTargets = new TargetConstraint[0];
        bodyColliderCache = null;
        collisionConstraint = null;
        groundCollisionConstraint = null;
        collisionParticles = new Particle[0];
        unsafeFinalCollisionCount = 0;
        rigidBodyCollisionAuditor = null;
        maximumFinalRigidBodyPenetration = 0;
        loopController = null;
        loopBakeConfig = null;
        loopErrorReport = null;
        loopSeamReport = null;
        bestLoopSeamCandidateReport = null;
        loopConverged = false;
        loopFallbackUsed = false;
        loopSeamCorrectionRejected = false;
        completedLoopCycles = 0;
        bestLoopCycleCandidate = null;
        cycleDt = dt;
        activeTransition = null;
        transitionController = null;
        transitionStarted = false;
        transitionPreRollSteps = 0;
        transitionSteps = 0;
        engine.clear();
    }

    private void closeRigidBodySession() {
        if (rigidBodySession == null) return;
        rigidBodySession.close();
        rigidBodySession = null;
    }

    public static final class BakedFrame {
        public final double time;
        public final List<BoneState> boneStates;
        private final Map<String, BoneState> boneStateCache = new HashMap<>();

        public BakedFrame(double time, List<BoneState> boneStates) {
            this.time = time;
            this.boneStates = List.copyOf(boneStates);
            for (BoneState state : this.boneStates) {
                if (state != null && state.boneName != null) {
                    boneStateCache.put(state.boneName, state);
                }
            }
        }

        public BoneState getBoneState(String boneName) {
            return boneStateCache.get(boneName);
        }
    }

    public static final class BoneState {
        public final String boneName;
        /** 局部动画位置，语义与导出的 JSON 一致。 */
        public final double[] position;
        public final double[] rotation;
        /** 模型空间线速度，单位为模型单位/秒。 */
        public final double[] linearVelocity;
        /** 模拟得到的模型/世界位置，保留用于诊断和校验。 */
        public final double[] worldPosition;

        public BoneState(String boneName, double[] position, double[] rotation) {
            this(boneName, position, rotation, new double[]{0, 0, 0});
        }

        public BoneState(String boneName, double[] position, double[] rotation,
                         double[] linearVelocity) {
            this(boneName, position, rotation, linearVelocity, null);
        }

        public BoneState(String boneName, double[] position, double[] rotation,
                         double[] linearVelocity, double[] worldPosition) {
            this.boneName = boneName;
            this.position = position;
            this.rotation = rotation;
            this.linearVelocity = linearVelocity;
            this.worldPosition = worldPosition;
        }
    }
}
