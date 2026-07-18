package xpbd.rigidbody;

import xpbd.baker.BakeProfiler;

import java.util.List;
import java.util.Objects;

/** 独立刚体烘焙后端的引擎无关边界接口。 */
public interface RigidBodyBackend extends AutoCloseable {
    enum SnapshotLevel {
        NONE,
        CONTACTS_ONLY,
        FULL_DIAGNOSTICS
    }

    enum MotionType {
        STATIC,
        KINEMATIC,
        DYNAMIC
    }

    record Transform(double[] translation, double[] rotation) {
        public Transform {
            translation = finiteVector(translation, 3, "translation");
            rotation = finiteVector(rotation, 4, "rotation");
            double lengthSquared = 0;
            for (double component : rotation) lengthSquared += component * component;
            if (lengthSquared <= 1e-20) {
                throw new IllegalArgumentException("rotation must be invertible");
            }
            double inverseLength = 1.0 / Math.sqrt(lengthSquared);
            for (int i = 0; i < rotation.length; i++) rotation[i] *= inverseLength;
        }

        public static Transform identity() {
            return new Transform(new double[]{0, 0, 0}, new double[]{0, 0, 0, 1});
        }

        @Override
        public double[] translation() {
            return translation.clone();
        }

        @Override
        public double[] rotation() {
            return rotation.clone();
        }
    }

    record BoxShape(double[] halfExtents, Transform localTransform) {
        public BoxShape {
            halfExtents = finiteVector(halfExtents, 3, "box half extents");
            for (double halfExtent : halfExtents) {
                if (!(halfExtent > 0)) {
                    throw new IllegalArgumentException(
                            "box half extents must be greater than zero");
                }
            }
            Objects.requireNonNull(localTransform, "localTransform");
        }

        @Override
        public double[] halfExtents() {
            return halfExtents.clone();
        }
    }

    record CcdSettings(boolean enabled, double motionThreshold,
                       double sweptSphereRadius) {
        public CcdSettings {
            if (!Double.isFinite(motionThreshold) || motionThreshold < 0
                    || !Double.isFinite(sweptSphereRadius)
                    || sweptSphereRadius < 0) {
                throw new IllegalArgumentException("CCD values must be finite and non-negative");
            }
            if (enabled && (!(motionThreshold > 0) || !(sweptSphereRadius > 0))) {
                throw new IllegalArgumentException(
                        "enabled CCD requires a positive threshold and radius");
            }
        }

        public static CcdSettings disabled() {
            return new CcdSettings(false, 0, 0);
        }
    }

    record BodyDefinition(String name, MotionType motionType, List<BoxShape> boxes,
                          Transform initialBoneTransform, double mass,
                          double friction, double restitution,
                          CcdSettings ccd) {
        public BodyDefinition {
            if (name == null || name.isBlank()) {
                throw new IllegalArgumentException("body name is required");
            }
            Objects.requireNonNull(motionType, "motionType");
            boxes = List.copyOf(Objects.requireNonNull(boxes, "boxes"));
            if (boxes.isEmpty() && motionType != MotionType.KINEMATIC) {
                throw new IllegalArgumentException(
                        "only a kinematic anchor may omit collision boxes");
            }
            Objects.requireNonNull(initialBoneTransform, "initialBoneTransform");
            if (!Double.isFinite(mass)
                    || (motionType == MotionType.DYNAMIC ? !(mass > 0) : mass != 0)) {
                throw new IllegalArgumentException(
                        "dynamic bodies require positive mass; other bodies require zero mass");
            }
            if (!Double.isFinite(friction) || friction < 0) {
                throw new IllegalArgumentException("friction must be finite and non-negative");
            }
            if (!Double.isFinite(restitution) || restitution < 0 || restitution > 1) {
                throw new IllegalArgumentException("restitution must be between zero and one");
            }
            Objects.requireNonNull(ccd, "ccd");
            if (motionType != MotionType.DYNAMIC && ccd.enabled()) {
                throw new IllegalArgumentException("CCD settings belong to dynamic bodies");
            }
        }
    }

    record BodyHandle(int id, String name) {
        public BodyHandle {
            if (id <= 0 || name == null || name.isBlank()) {
                throw new IllegalArgumentException("invalid body handle");
            }
        }
    }

    record BodyState(Transform boneTransform, double[] linearVelocity,
                     double[] angularVelocity) {
        public BodyState {
            Objects.requireNonNull(boneTransform, "boneTransform");
            linearVelocity = finiteVector(linearVelocity, 3, "linear velocity");
            angularVelocity = finiteVector(angularVelocity, 3, "angular velocity");
        }

        @Override
        public double[] linearVelocity() {
            return linearVelocity.clone();
        }

        @Override
        public double[] angularVelocity() {
            return angularVelocity.clone();
        }
    }

    record JointSettings(double[] angularLowerLimit, double[] angularUpperLimit,
                         double stiffness, double damping) {
        public JointSettings {
            angularLowerLimit = finiteVector(
                    angularLowerLimit, 3, "angular lower limit");
            angularUpperLimit = finiteVector(
                    angularUpperLimit, 3, "angular upper limit");
            for (int axis = 0; axis < 3; axis++) {
                if (angularLowerLimit[axis] > angularUpperLimit[axis]) {
                    throw new IllegalArgumentException(
                            "angular lower limit exceeds upper limit");
                }
            }
            if (!Double.isFinite(stiffness) || stiffness < 0
                    || !Double.isFinite(damping) || damping < 0) {
                throw new IllegalArgumentException(
                        "joint stiffness and damping must be finite and non-negative");
            }
        }

        @Override
        public double[] angularLowerLimit() {
            return angularLowerLimit.clone();
        }

        @Override
        public double[] angularUpperLimit() {
            return angularUpperLimit.clone();
        }
    }

    record SweepResult(boolean hit, double hitFraction, String hitBodyName) {
        public SweepResult {
            if (!Double.isFinite(hitFraction) || hitFraction < 0 || hitFraction > 1) {
                throw new IllegalArgumentException("hit fraction must be between zero and one");
            }
            if (!hit) hitBodyName = null;
        }

        public static SweepResult miss() {
            return new SweepResult(false, 1, null);
        }
    }

    /** 一个固定步后捕获的不可变 Bullet 接触流形证据。 */
    record ContactSnapshot(
            BodyHandle bodyA, BodyHandle bodyB,
            int partIdA, int shapeIndexA, int partIdB, int shapeIndexB,
            double[] pointOnA, double[] pointOnB, double[] normalOnB,
            double penetration, int lifetime, double appliedImpulse,
            double relativeNormalVelocityBefore,
            double[] relativeTangentVelocityBefore,
            double relativeNormalVelocityAfter,
            double[] relativeTangentVelocityAfter,
            BodyState bodyABefore, BodyState bodyAAfter,
            BodyState bodyBBefore, BodyState bodyBAfter) {
        public ContactSnapshot {
            Objects.requireNonNull(bodyA, "bodyA");
            Objects.requireNonNull(bodyB, "bodyB");
            pointOnA = finiteVector(pointOnA, 3, "contact point on A");
            pointOnB = finiteVector(pointOnB, 3, "contact point on B");
            normalOnB = finiteVector(normalOnB, 3, "contact normal on B");
            relativeTangentVelocityBefore = finiteVector(
                    relativeTangentVelocityBefore, 3,
                    "relative tangent velocity before");
            relativeTangentVelocityAfter = finiteVector(
                    relativeTangentVelocityAfter, 3,
                    "relative tangent velocity after");
            if (!Double.isFinite(penetration) || penetration < 0
                    || !Double.isFinite(appliedImpulse)
                    || !Double.isFinite(relativeNormalVelocityBefore)
                    || !Double.isFinite(relativeNormalVelocityAfter)) {
                throw new IllegalArgumentException(
                        "contact diagnostics must be finite and non-negative where applicable");
            }
            Objects.requireNonNull(bodyABefore, "bodyABefore");
            Objects.requireNonNull(bodyAAfter, "bodyAAfter");
            Objects.requireNonNull(bodyBBefore, "bodyBBefore");
            Objects.requireNonNull(bodyBAfter, "bodyBAfter");
        }

        @Override public double[] pointOnA() { return pointOnA.clone(); }
        @Override public double[] pointOnB() { return pointOnB.clone(); }
        @Override public double[] normalOnB() { return normalOnB.clone(); }
        @Override public double[] relativeTangentVelocityBefore() {
            return relativeTangentVelocityBefore.clone();
        }
        @Override public double[] relativeTangentVelocityAfter() {
            return relativeTangentVelocityAfter.clone();
        }
    }

    /** 一个实际 Bullet 复合子盒体的世界空间变换。 */
    record BodyShapeSnapshot(BodyHandle body, int shapeIndex,
                             double[] halfExtents, Transform worldTransform) {
        public BodyShapeSnapshot {
            Objects.requireNonNull(body, "body");
            halfExtents = finiteVector(halfExtents, 3, "shape half extents");
            Objects.requireNonNull(worldTransform, "worldTransform");
        }

        @Override public double[] halfExtents() { return halfExtents.clone(); }
    }

    record NamedBodyState(BodyHandle body, BodyState state) {
        public NamedBodyState {
            Objects.requireNonNull(body, "body");
            Objects.requireNonNull(state, "state");
        }
    }

    BodyHandle createBody(BodyDefinition definition);

    /** 创建无限静态 XZ 平面，其朝上表面位于 Y=height。 */
    BodyHandle createGroundPlane(String name, double height,
                                 double friction, double restitution);

    void addSpringJoint(BodyHandle bodyA, BodyHandle bodyB, Transform worldAnchor,
                        JointSettings settings);

    SweepResult sweepKinematic(BodyHandle body, Transform fromBoneTransform,
                               Transform toBoneTransform);

    void setKinematicTransform(BodyHandle body, Transform boneTransform,
                               double dt, boolean continuousHistory);

    /** 为下一个固定步累积世界空间力。 */
    void applyCentralForce(BodyHandle body, double[] force);

    void step(double fixedDt);

    /** 配置可选证据实体化；默认保留完整诊断信息。 */
    default void setSnapshotLevel(SnapshotLevel level) {
    }

    /** 将后端连接至所属烘焙性能分析器。 */
    default void setProfiler(BakeProfiler profiler) {
    }

    BodyState getBodyState(BodyHandle body);

    int getContactCount();

    double getMaximumPenetration();

    default List<ContactSnapshot> getContactSnapshots() {
        return List.of();
    }

    default List<BodyShapeSnapshot> getBodyShapeSnapshots() {
        return List.of();
    }

    default List<NamedBodyState> getBodyStateSnapshots() {
        return List.of();
    }

    int getNativeBulletVersion();

    @Override
    void close();

    private static double[] finiteVector(double[] value, int length, String label) {
        if (value == null || value.length < length) {
            throw new IllegalArgumentException(label + " needs " + length + " components");
        }
        double[] result = new double[length];
        for (int i = 0; i < length; i++) {
            if (!Double.isFinite(value[i])) {
                throw new IllegalArgumentException(label + " must be finite");
            }
            result[i] = value[i];
        }
        return result;
    }
}
