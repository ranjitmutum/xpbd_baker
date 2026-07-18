package xpbd.baker;

import xpbd.loader.BedrockModelData;
import xpbd.rigidbody.RigidBodyBackend;

import java.util.*;

public final class BoneMapper {
    public enum SimulationMode {
        XPBD,
        RIGID_BODY
    }

    public enum LoopMode {
        AUTO,
        FORCE_LOOP,
        FORCE_ONCE
    }

    public enum LoopSeamStrategy {
    /** 保留固定/源动画驱动，仅闭合次级物理运动。 */
        PHYSICS_RELATIVE,
    /** 允许对固定根骨骼做小幅修正，以闭合完整可视子树。 */
        VISUAL_SUBTREE
    }

    private final List<BedrockModelData.Bone> allBones = new ArrayList<>();
    private final List<BedrockModelData.Bone> allBonesView =
            Collections.unmodifiableList(allBones);
    private final Map<String, BedrockModelData.Bone> bonesByName = new HashMap<>();
    private Map<String, BonePoseCalculator.Pose> restPoses = Collections.emptyMap();
    private final Set<String> physicsBones = new LinkedHashSet<>();
    private final Set<String> collisionRoots = new LinkedHashSet<>();
    private final Map<String, Integer> boneToParticle = new HashMap<>();
    private final Map<String, BonePhysicsConfig> perBoneConfigs = new HashMap<>();

    private final PhysicsGroupConfig config = new PhysicsGroupConfig();

    public BoneMapper(List<BedrockModelData.Bone> allBones) {
        replaceModelBones(allBones);
    }

    public List<BedrockModelData.Bone> getAllBones() {
        return allBonesView;
    }

    /** 原子替换模型数据，并使全部模型相关选择和缓存失效。 */
    public void replaceModelBones(Collection<BedrockModelData.Bone> bones) {
        List<BedrockModelData.Bone> replacement = bones == null
                ? Collections.emptyList() : new ArrayList<>(bones);
        clearModelState();
        allBones.clear();
        allBones.addAll(replacement);
        refreshModelCache();
    }

    public void addPhysicsBone(String boneName) {
        if (boneName != null && bonesByName.containsKey(boneName)) {
            physicsBones.add(boneName);
        }
    }

    public void removePhysicsBone(String boneName) {
        physicsBones.remove(boneName);
    }

    public boolean isPhysicsBone(String boneName) {
        return physicsBones.contains(boneName);
    }

    public Set<String> getPhysicsBones() {
        return Collections.unmodifiableSet(physicsBones);
    }

    public void addCollisionRoot(String boneName) {
        if (boneName != null && bonesByName.containsKey(boneName)) {
            collisionRoots.add(boneName);
        }
    }

    public void removeCollisionRoot(String boneName) {
        collisionRoots.remove(boneName);
    }

    public void clearCollisionRoots() {
        collisionRoots.clear();
    }

    public boolean isCollisionRoot(String boneName) {
        return collisionRoots.contains(boneName);
    }

    public Set<String> getCollisionRoots() {
        return Collections.unmodifiableSet(collisionRoots);
    }

    /**
     * 为一次烘焙展开选定根节点。物理骨骼及其全部后代会被排除，
     * 因为其变换不再是纯参考动画，否则会与自身模拟子树发生碰撞。
     */
    public Set<String> getExpandedCollisionBones() {
        if (collisionRoots.isEmpty()) return Collections.emptySet();
        Map<String, List<String>> children = new HashMap<>();
        for (BedrockModelData.Bone bone : allBones) {
            if (bone != null && bone.name != null && bone.parent != null) {
                children.computeIfAbsent(bone.parent, ignored -> new ArrayList<>())
                        .add(bone.name);
            }
        }
        Set<String> reachable = new HashSet<>();
        Deque<String> pending = new ArrayDeque<>(collisionRoots);
        while (!pending.isEmpty()) {
            String name = pending.removeFirst();
            if (!reachable.add(name)) continue;
            pending.addAll(children.getOrDefault(name, Collections.emptyList()));
        }
        Set<String> result = new LinkedHashSet<>();
        for (BedrockModelData.Bone bone : allBones) {
            if (bone != null && reachable.contains(bone.name)
                    && !hasPhysicsAncestorOrSelf(bone.name)) {
                result.add(bone.name);
            }
        }
        return Collections.unmodifiableSet(result);
    }

    private boolean hasPhysicsAncestorOrSelf(String boneName) {
        Set<String> visited = new HashSet<>();
        BedrockModelData.Bone bone = bonesByName.get(boneName);
        while (bone != null && visited.add(bone.name)) {
            if (physicsBones.contains(bone.name)) return true;
            bone = bone.parent == null ? null : bonesByName.get(bone.parent);
        }
        return false;
    }

    /** 清除全部与当前加载模型绑定的状态。 */
    public void resetModelState() {
        clearModelState();
        refreshModelCache();
    }

    private void clearModelState() {
        physicsBones.clear();
        collisionRoots.clear();
        boneToParticle.clear();
        perBoneConfigs.clear();
    }

    public PhysicsGroupConfig getConfig() {
        return config;
    }

    public double[] getWorldPivot(String boneName) {
        BonePoseCalculator.Pose pose = restPoses.get(boneName);
        return pose == null ? new double[]{0, 0, 0} : pose.worldPosition.clone();
    }

    public void buildParticleMapping() {
        refreshModelCache();
        boneToParticle.clear();
        int idx = 0;
        for (String name : physicsBones) {
            boneToParticle.put(name, idx++);
        }
    }

    private void refreshModelCache() {
        bonesByName.clear();
        for (BedrockModelData.Bone bone : allBones) {
            if (bone != null && bone.name != null) bonesByName.put(bone.name, bone);
        }
        restPoses = BonePoseCalculator.calculate(allBones, null, 0);
    }

    public int getParticleIndex(String boneName) {
        return boneToParticle.getOrDefault(boneName, -1);
    }

    /**
     * 在物理组的父子骨骼对之间自动生成距离约束。
     */
    public List<ConstraintDef> generateChainConstraints() {
        List<ConstraintDef> constraints = new ArrayList<>();
        for (String boneName : physicsBones) {
            BedrockModelData.Bone bone = bonesByName.get(boneName);
            if (bone == null) continue;
            if (bone.parent != null && physicsBones.contains(bone.parent)) {
                double[] posA = getWorldPivot(bone.parent);
                double[] posB = getWorldPivot(boneName);
                double dx = posB[0] - posA[0];
                double dy = posB[1] - posA[1];
                double dz = posB[2] - posA[2];
                double restLen = Math.sqrt(dx * dx + dy * dy + dz * dz);
        // 组织/控制骨骼通常共享枢轴。保留零长度焊接约束：改为 1 会产生剧烈跳变，
        // 跳过则会让子骨骼成为自由粒子并下落。
                if (restLen < 1e-6) restLen = 0;
                constraints.add(new ConstraintDef(
                        bone.parent, boneName, restLen,
                        getEffectiveCompliance(boneName),
                        getEffectiveDampingCompliance(boneName)));
            }
        }

        // 分叉父节点与两个子节点组成可见刚性扇形。仅有父子距离会让同级骨骼独立绕转，
        // 在湍流下撕裂扇形，因此用同级距离边闭合每个分支。
        Map<String, List<String>> childrenByParent = new LinkedHashMap<>();
        for (String boneName : physicsBones) {
            BedrockModelData.Bone bone = bonesByName.get(boneName);
            if (bone != null && bone.parent != null && physicsBones.contains(bone.parent)) {
                childrenByParent.computeIfAbsent(bone.parent, ignored -> new ArrayList<>())
                        .add(boneName);
            }
        }
        for (List<String> children : childrenByParent.values()) {
            for (int i = 0; i < children.size(); i++) {
                for (int j = i + 1; j < children.size(); j++) {
                    String a = children.get(i);
                    String b = children.get(j);
                    double[] posA = getWorldPivot(a);
                    double[] posB = getWorldPivot(b);
                    double dx = posB[0] - posA[0];
                    double dy = posB[1] - posA[1];
                    double dz = posB[2] - posA[2];
                    double restLen = Math.sqrt(dx * dx + dy * dy + dz * dz);
                    if (restLen < 1e-6) restLen = 0;
                    constraints.add(new ConstraintDef(a, b, restLen,
                            Math.min(getEffectiveCompliance(a), getEffectiveCompliance(b)),
                            Math.min(getEffectiveDampingCompliance(a),
                                    getEffectiveDampingCompliance(b))));
                }
            }
        }
        return constraints;
    }

    /** 为每组连续物理骨骼三元组建立稳定的外点跨度范围。 */
    public List<CrossSpringDef> generateCrossSpringConstraints() {
        List<CrossSpringDef> constraints = new ArrayList<>();
        if (!config.enableAngleConstraints) return constraints;
        for (String childName : physicsBones) {
            BedrockModelData.Bone child = bonesByName.get(childName);
            if (child == null || child.parent == null || !physicsBones.contains(child.parent)) continue;
            BedrockModelData.Bone joint = bonesByName.get(child.parent);
            if (joint == null || joint.parent == null || !physicsBones.contains(joint.parent)) continue;
            double[] a = getWorldPivot(joint.parent);
            double[] b = getWorldPivot(joint.name);
            double[] c = getWorldPivot(childName);
            double ux = a[0]-b[0], uy = a[1]-b[1], uz = a[2]-b[2];
            double vx = c[0]-b[0], vy = c[1]-b[1], vz = c[2]-b[2];
            double lu = Math.sqrt(ux*ux + uy*uy + uz*uz);
            double lv = Math.sqrt(vx*vx + vy*vy + vz*vz);
            if (lu < 1e-6 || lv < 1e-6) continue;
            double cosine = Math.max(-1, Math.min(1, (ux*vx + uy*vy + uz*vz)/(lu*lv)));
            double restAngle = Math.acos(cosine);
            double deviation = Math.toRadians(getEffectiveMaxBendDegrees(joint.name));
            double minAngle = Math.max(0, restAngle - deviation);
            double maxAngle = Math.min(Math.PI, restAngle + deviation);
            double minSpan = spanForAngle(lu, lv, minAngle);
            double maxSpan = spanForAngle(lu, lv, maxAngle);
            constraints.add(new CrossSpringDef(joint.parent, childName,
                    minSpan, maxSpan, getEffectiveBendCompliance(joint.name),
                    c[0] - a[0], c[1] - a[1], c[2] - a[2]));
        }
        return constraints;
    }

    private static double spanForAngle(double lengthA, double lengthB, double angle) {
        double squared = lengthA * lengthA + lengthB * lengthB
                - 2.0 * lengthA * lengthB * Math.cos(angle);
        return Math.sqrt(Math.max(0, squared));
    }

    public static class PhysicsGroupConfig {
    /** XPBD 仍是现有项目的兼容性默认值。 */
        public SimulationMode simulationMode = SimulationMode.XPBD;
        public double particleMass = 1.0;
        public double compliance = 0.000001;
        public double dampingCompliance = 0.00001;
        public boolean enableAngleConstraints = true;
    /** 相对于模型静止角度的最大角偏差。 */
        public double maxBendDegrees = 75.0;
        public double bendCompliance = 0.00001;
        public double gravityY = -9.8;
    /** 启用时，动态骨骼不再受到动画目标牵引。 */
        public boolean enableRealGravityField = false;
    /** 可选的 Y=0 无限 XZ 地面平面；默认关闭以保持兼容性。 */
        public boolean enableGroundCollision = false;
        public int solverIterations = 8;
        public double animationPullCompliance = 0.1;
    /** 身体立方体碰撞使用的模拟枢轴点半径。 */
        public double collisionSkin = 0.1;
    /** XPBD 法向速度恢复系数，与 Bullet 恢复系数独立。 */
        public double xpbdCollisionRestitution = 0.0;
        public double windSpeed = 6.0;
    /** 水平方位角：0 = +X，90 = +Z。 */
        public double windDirectionDegrees = 20.0;
    /** 垂直仰角：正值指向上方。 */
        public double windElevationDegrees = 20.0;
    /** 为 true 时，windX/Y/Z 将替代速度、方向和仰角。 */
        public boolean useWindComponents = false;
        public double windX = 0.0;
        public double windY = 0.0;
        public double windZ = 0.0;
    /** Bedrock 动画 JSON 通常不包含的实体/世界运动。 */
        public double movementSpeed = 0.0;
        public double movementDirectionDegrees = 0.0;
        public double movementElevationDegrees = 0.0;
        public double airDrag = 2.0;
        public double turbulence = 1.5;
    /** 在片段两端将物理骨骼混合到参考动画所用的秒数。 */
        public double transitionDuration = 0.25;
        public LoopMode loopMode = LoopMode.AUTO;
    /** 循环可被视为稳定前所需的最少完整周期数。 */
        public int minimumWarmupCycles = 2;
    /** 自适应周期不动点迭代的硬性预算。 */
        public int maximumWarmupCycles = 12;
    /** 必须通过所有阈值的连续边界比较次数。 */
        public int requiredStableCycles = 2;
        public double loopPositionTolerance = 0.001;
        public double loopRotationToleranceDegrees = 0.1;
        public double loopLinearVelocityTolerance = 0.01;
        public double loopAngularVelocityTolerance = 0.01;
    /** 若收敛耗尽预算，则显式缝合最后一个周期。 */
        public boolean loopSeamFallbackEnabled = true;
        public LoopSeamStrategy loopSeamStrategy = LoopSeamStrategy.PHYSICS_RELATIVE;
    /** 首个修正窗口；不安全候选会扩展到 0.375 和 0.5。 */
        public double loopSeamWindowRatio = 0.25;
        public boolean loopSeamMatchAcceleration = true;
        public double loopSeamRelativeVelocityTolerance = 0.02;
        public double loopSeamMinimumLinearVelocityTolerance = 0.01;
        public double loopSeamMinimumAngularVelocityTolerance = 0.01;
        public double loopSeamRelativeAccelerationTolerance = 0.05;
    /** 一个输出步中的 Bullet 子步数；2 表示以 60 Hz 输出进行 120 Hz 求解。 */
        public int rigidBodySubsteps = 2;
    /** 除非明确降级，否则保留每个子步的证据供界面/测试使用。 */
        public RigidBodyBackend.SnapshotLevel rigidBodySnapshotLevel =
                RigidBodyBackend.SnapshotLevel.FULL_DIAGNOSTICS;
    /** 将 Bedrock 模型单位（像素）转换为 Bullet 世界单位（方块）。 */
        public double rigidBodyUnitScale = 1.0 / 16.0;
        public double rigidBodyJointStiffness = 12.0;
        public double rigidBodyJointDamping = 0.8;
    /** 仅刚体关节使用的对称局部轴角度限制。 */
        public double rigidBodyMaxBendXDegrees = 75.0;
        public double rigidBodyMaxBendYDegrees = 75.0;
        public double rigidBodyMaxBendZDegrees = 75.0;
        public double rigidBodyFriction = 0.5;
        public double rigidBodyRestitution = 0.0;
        public boolean rigidBodyCcd = true;
    /** 当 Bullet 报告的穿透深度超过该世界单位值时禁止导出。 */
        public double rigidBodyMaximumSafePenetration = 0.2;
    }

    public static class BonePhysicsConfig {
        public Double particleMass;
        public Double compliance;
        public Double dampingCompliance;
        public Double maxBendDegrees;
        public Double bendCompliance;
        public Double rigidBodyMaxBendXDegrees;
        public Double rigidBodyMaxBendYDegrees;
        public Double rigidBodyMaxBendZDegrees;
        public Double animationPullCompliance;
        public Double gravityScale;
        public Double windInfluence;
        public Double turbulenceInfluence;
        public Boolean fixed;
    }

    public BonePhysicsConfig getBoneConfig(String boneName) {
        return perBoneConfigs.get(boneName);
    }

    public void setBoneConfig(String boneName, BonePhysicsConfig cfg) {
        if (boneName == null || !bonesByName.containsKey(boneName)) return;
        if (cfg == null) {
            perBoneConfigs.remove(boneName);
        } else {
            perBoneConfigs.put(boneName, cfg);
        }
    }

    public double getEffectiveMass(String boneName) {
        BonePhysicsConfig bc = perBoneConfigs.get(boneName);
        return effectiveNonNegative(bc != null ? bc.particleMass : null,
                config.particleMass, 1.0);
    }

    public double getEffectiveCompliance(String boneName) {
        BonePhysicsConfig bc = perBoneConfigs.get(boneName);
        return effectiveNonNegative(bc != null ? bc.compliance : null,
                config.compliance, 0);
    }

    public double getEffectiveDampingCompliance(String boneName) {
        BonePhysicsConfig bc = perBoneConfigs.get(boneName);
        return effectiveNonNegative(bc != null ? bc.dampingCompliance : null,
                config.dampingCompliance, 0);
    }

    public double getEffectiveMaxBendDegrees(String boneName) {
        BonePhysicsConfig bc = perBoneConfigs.get(boneName);
        double value = (bc != null && bc.maxBendDegrees != null)
                ? bc.maxBendDegrees : config.maxBendDegrees;
        return Double.isFinite(value) ? Math.max(0, Math.min(180, value)) : 180;
    }

    public double getEffectiveBendCompliance(String boneName) {
        BonePhysicsConfig bc = perBoneConfigs.get(boneName);
        double value = (bc != null && bc.bendCompliance != null)
                ? bc.bendCompliance : config.bendCompliance;
        return Double.isFinite(value) ? Math.max(0, value) : 0;
    }

    public double[] getEffectiveRigidBodyMaxBendDegrees(String boneName) {
        BonePhysicsConfig bc = perBoneConfigs.get(boneName);
        return new double[]{
                effectiveAngle(bc != null ? bc.rigidBodyMaxBendXDegrees : null,
                        config.rigidBodyMaxBendXDegrees),
                effectiveAngle(bc != null ? bc.rigidBodyMaxBendYDegrees : null,
                        config.rigidBodyMaxBendYDegrees),
                effectiveAngle(bc != null ? bc.rigidBodyMaxBendZDegrees : null,
                        config.rigidBodyMaxBendZDegrees)
        };
    }

    private static double effectiveAngle(Double override, double fallback) {
        double value = override != null ? override : fallback;
        return Double.isFinite(value) ? Math.max(0, Math.min(180, value)) : 180;
    }

    public double getEffectiveAnimPullCompliance(String boneName) {
        if (config.enableRealGravityField) return 0;
        BonePhysicsConfig bc = perBoneConfigs.get(boneName);
        return effectiveNonNegative(bc != null ? bc.animationPullCompliance : null,
                config.animationPullCompliance, 0);
    }

    public double getEffectiveGravityScale(String boneName) {
        BonePhysicsConfig bc = perBoneConfigs.get(boneName);
        return effectiveNonNegative(bc != null ? bc.gravityScale : null, 1.0, 1.0);
    }

    public double getEffectiveWindInfluence(String boneName) {
        BonePhysicsConfig bc = perBoneConfigs.get(boneName);
        return effectiveNonNegative(bc != null ? bc.windInfluence : null, 1.0, 1.0);
    }

    public double getEffectiveTurbulenceInfluence(String boneName) {
        BonePhysicsConfig bc = perBoneConfigs.get(boneName);
        return effectiveNonNegative(bc != null ? bc.turbulenceInfluence : null,
                1.0, 1.0);
    }

    public boolean isFixedBone(String boneName) {
        BonePhysicsConfig bc = perBoneConfigs.get(boneName);
        if (bc != null && bc.fixed != null) return bc.fixed;
        // 真实重力模式为可选自由落体：仅显式固定的骨骼保留运动学锚点；
        // 辅助模式保持旧版根骨骼自动固定行为。
        if (config.enableRealGravityField) return false;
        // 默认：自动识别为链根（父骨骼不在物理集合中）。
        BedrockModelData.Bone bone = bonesByName.get(boneName);
        if (bone == null || bone.parent == null) return true;
        return !physicsBones.contains(bone.parent);
    }

    private static double effectiveNonNegative(Double override, double fallback,
                                               double defaultValue) {
        if (override != null && Double.isFinite(override)) return Math.max(0, override);
        return Double.isFinite(fallback) ? Math.max(0, fallback) : defaultValue;
    }

    public static class ConstraintDef {
        public final String boneA;
        public final String boneB;
        public final double restLength;
        public final double compliance;
        public final double dampingCompliance;

        public ConstraintDef(String boneA, String boneB, double restLength,
                             double compliance, double dampingCompliance) {
            this.boneA = boneA;
            this.boneB = boneB;
            this.restLength = restLength;
            this.compliance = compliance;
            this.dampingCompliance = dampingCompliance;
        }
    }

    public static class CrossSpringDef {
        public final String boneA;
        public final String boneC;
        public final double minDistance;
        public final double maxDistance;
        public final double compliance;
        public final double fallbackX;
        public final double fallbackY;
        public final double fallbackZ;

        public CrossSpringDef(String boneA, String boneC,
                              double minDistance, double maxDistance,
                              double compliance, double fallbackX,
                              double fallbackY, double fallbackZ) {
            this.boneA = boneA;
            this.boneC = boneC;
            this.minDistance = minDistance;
            this.maxDistance = maxDistance;
            this.compliance = compliance;
            this.fallbackX = fallbackX;
            this.fallbackY = fallbackY;
            this.fallbackZ = fallbackZ;
        }
    }
}
