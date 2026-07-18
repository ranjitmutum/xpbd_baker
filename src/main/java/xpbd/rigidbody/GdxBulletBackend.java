package xpbd.rigidbody;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.Bullet;
import com.badlogic.gdx.physics.bullet.collision.ClosestNotMeConvexResultCallback;
import com.badlogic.gdx.physics.bullet.collision.CollisionConstants;
import com.badlogic.gdx.physics.bullet.collision.btBoxShape;
import com.badlogic.gdx.physics.bullet.collision.btCollisionShape;
import com.badlogic.gdx.physics.bullet.collision.btCollisionDispatcher;
import com.badlogic.gdx.physics.bullet.collision.btCollisionObject;
import com.badlogic.gdx.physics.bullet.collision.btCompoundShape;
import com.badlogic.gdx.physics.bullet.collision.btDbvtBroadphase;
import com.badlogic.gdx.physics.bullet.collision.btDefaultCollisionConfiguration;
import com.badlogic.gdx.physics.bullet.collision.btEmptyShape;
import com.badlogic.gdx.physics.bullet.collision.btManifoldPoint;
import com.badlogic.gdx.physics.bullet.collision.btPersistentManifold;
import com.badlogic.gdx.physics.bullet.collision.btStaticPlaneShape;
import com.badlogic.gdx.physics.bullet.dynamics.btDiscreteDynamicsWorld;
import com.badlogic.gdx.physics.bullet.dynamics.btGeneric6DofSpring2Constraint;
import com.badlogic.gdx.physics.bullet.dynamics.btRigidBody;
import com.badlogic.gdx.physics.bullet.dynamics.btSequentialImpulseConstraintSolver;
import com.badlogic.gdx.utils.BufferUtils;
import xpbd.baker.BakeProfiler;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/** 隐藏在引擎无关烘焙边界后的 gdx-bullet 实现。 */
public final class GdxBulletBackend implements RigidBodyBackend {
    private final btDefaultCollisionConfiguration collisionConfiguration;
    private final btCollisionDispatcher dispatcher;
    private final btDbvtBroadphase broadphase;
    private final btSequentialImpulseConstraintSolver solver;
    private final btDiscreteDynamicsWorld world;
    private final Map<Integer, BodyEntry> bodies = new LinkedHashMap<>();
    private final Map<Integer, String> bodyNames = new HashMap<>();
    private final List<btGeneric6DofSpring2Constraint> constraints = new ArrayList<>();
    private List<ContactSnapshot> contactSnapshots = List.of();
    private SnapshotLevel snapshotLevel = SnapshotLevel.FULL_DIAGNOSTICS;
    private BakeProfiler profiler = BakeProfiler.disabled();
    private int nextBodyId = 1;
    private boolean closed;

    public GdxBulletBackend(double gravityX, double gravityY, double gravityZ) {
        if (!Double.isFinite(gravityX) || !Double.isFinite(gravityY)
                || !Double.isFinite(gravityZ)) {
            throw new IllegalArgumentException("gravity must be finite");
        }
        Bullet.init();
        collisionConfiguration = new btDefaultCollisionConfiguration();
        dispatcher = new btCollisionDispatcher(collisionConfiguration);
        broadphase = new btDbvtBroadphase();
        solver = new btSequentialImpulseConstraintSolver();
        solver.setRandSeed(0);
        world = new btDiscreteDynamicsWorld(
                dispatcher, broadphase, solver, collisionConfiguration);
        world.setGravity(vector(gravityX, gravityY, gravityZ));
        world.setForceUpdateAllAabbs(true);
    }

    @Override
    public void setSnapshotLevel(SnapshotLevel level) {
        requireOpen();
        snapshotLevel = Objects.requireNonNull(level, "level");
        if (level == SnapshotLevel.NONE) contactSnapshots = List.of();
    }

    @Override
    public void setProfiler(BakeProfiler profiler) {
        requireOpen();
        this.profiler = Objects.requireNonNull(profiler, "profiler");
    }

    @Override
    public BodyHandle createBody(BodyDefinition definition) {
        requireOpen();
        Objects.requireNonNull(definition, "definition");

        btCompoundShape compound = definition.boxes().isEmpty()
                ? null : new btCompoundShape();
        List<btBoxShape> childShapes = new ArrayList<>(definition.boxes().size());
        double totalVolume = 0;
        for (BoxShape box : definition.boxes()) {
            double[] half = box.halfExtents();
            totalVolume += half[0] * half[1] * half[2] * 8.0;
            btBoxShape child = new btBoxShape(vector(half));
            child.setMargin((float) Math.min(0.04,
                    Math.min(half[0], Math.min(half[1], half[2])) * 0.2));
            if (compound != null) {
                compound.addChildShape(matrix(box.localTransform()), child);
            }
            childShapes.add(child);
        }

        Matrix4 boneToCenterOfMass = new Matrix4();
        Vector3 principalInertia = new Vector3();
        btCollisionShape collisionShape;
        if (compound == null) {
            collisionShape = new btEmptyShape();
        } else {
            FloatBuffer masses = BufferUtils.newFloatBuffer(childShapes.size());
            for (BoxShape box : definition.boxes()) {
                double[] half = box.halfExtents();
                double volume = half[0] * half[1] * half[2] * 8.0;
                double totalMass = definition.motionType() == MotionType.DYNAMIC
                        ? definition.mass() : 1.0;
                masses.put((float) (totalMass * volume / totalVolume));
            }
            masses.flip();
            compound.calculatePrincipalAxisTransform(
                    masses, boneToCenterOfMass, principalInertia);
            Matrix4 inverseCenterOfMass = boneToCenterOfMass.cpy().inv();
            for (int i = 0; i < compound.getNumChildShapes(); i++) {
                Matrix4 childTransform = compound.getChildTransform(i).cpy();
                compound.updateChildTransform(i,
                        inverseCenterOfMass.cpy().mul(childTransform), false);
            }
            compound.recalculateLocalAabb();
            collisionShape = compound;
        }
        Matrix4 centerOfMassToBone = boneToCenterOfMass.cpy().inv();

        Vector3 inertia = definition.motionType() == MotionType.DYNAMIC
                ? principalInertia : new Vector3();
        Matrix4 initialBodyTransform = matrix(definition.initialBoneTransform())
                .mul(boneToCenterOfMass);
        btRigidBody body = new btRigidBody((float) definition.mass(), null,
                collisionShape, inertia);
        body.setWorldTransform(initialBodyTransform);
        body.setInterpolationWorldTransform(initialBodyTransform);
        body.setFriction((float) definition.friction());
        body.setRestitution((float) definition.restitution());
        body.setDamping(0.02f, 0.05f);

        if (definition.motionType() == MotionType.KINEMATIC) {
            body.setCollisionFlags(body.getCollisionFlags()
                    | btCollisionObject.CollisionFlags.CF_KINEMATIC_OBJECT);
            body.setActivationState(CollisionConstants.DISABLE_DEACTIVATION);
        } else if (definition.motionType() == MotionType.DYNAMIC) {
            body.setActivationState(CollisionConstants.DISABLE_DEACTIVATION);
            if (definition.ccd().enabled()) {
                body.setCcdMotionThreshold((float) definition.ccd().motionThreshold());
                body.setCcdSweptSphereRadius(
                        (float) definition.ccd().sweptSphereRadius());
            }
        }

        int id = nextBodyId++;
        body.setUserValue(id);
        BodyEntry entry = new BodyEntry(definition.motionType(), body,
                collisionShape, compound,
                childShapes, definition.boxes().stream()
                        .map(BoxShape::halfExtents).toList(),
                boneToCenterOfMass.cpy(), centerOfMassToBone,
                matrix(definition.initialBoneTransform()));
        bodies.put(id, entry);
        bodyNames.put(id, definition.name());
        world.addRigidBody(body);
        world.updateSingleAabb(body);
        profiler.addCounter(BakeProfiler.Counter.BODY_COUNT, 1);
        if (definition.motionType() == MotionType.DYNAMIC) {
            profiler.addCounter(BakeProfiler.Counter.DYNAMIC_BODY_COUNT, 1);
        } else if (definition.motionType() == MotionType.KINEMATIC) {
            profiler.addCounter(BakeProfiler.Counter.KINEMATIC_BODY_COUNT, 1);
        }
        if (definition.boxes().size() > 1) {
            profiler.addCounter(BakeProfiler.Counter.COMPOUND_BODY_COUNT, 1);
        }
        return new BodyHandle(id, definition.name());
    }

    @Override
    public BodyHandle createGroundPlane(String name, double height,
                                        double friction, double restitution) {
        requireOpen();
        if (name == null || name.isBlank()) {
            throw new IllegalArgumentException("ground body name is required");
        }
        if (!Double.isFinite(height) || !Double.isFinite(friction) || friction < 0
                || !Double.isFinite(restitution) || restitution < 0
                || restitution > 1) {
            throw new IllegalArgumentException("invalid ground plane parameters");
        }
        btStaticPlaneShape shape = new btStaticPlaneShape(
                new Vector3(0, 1, 0), (float) height);
        Matrix4 identity = new Matrix4();
        btRigidBody body = new btRigidBody(0, null, shape, new Vector3());
        body.setWorldTransform(identity);
        body.setInterpolationWorldTransform(identity);
        body.setFriction((float) friction);
        body.setRestitution((float) restitution);

        int id = nextBodyId++;
        body.setUserValue(id);
        bodies.put(id, new BodyEntry(MotionType.STATIC, body, shape, null,
                List.of(), List.of(), identity.cpy(), identity.cpy(), identity.cpy()));
        bodyNames.put(id, name);
        world.addRigidBody(body);
        world.updateSingleAabb(body);
        return new BodyHandle(id, name);
    }

    @Override
    public void addSpringJoint(BodyHandle bodyA, BodyHandle bodyB,
                               Transform worldAnchor, JointSettings settings) {
        requireOpen();
        BodyEntry a = entry(bodyA);
        BodyEntry b = entry(bodyB);
        Matrix4 anchor = matrix(Objects.requireNonNull(worldAnchor, "worldAnchor"));
        Matrix4 frameA = a.body.getWorldTransform().cpy().inv().mul(anchor);
        Matrix4 frameB = b.body.getWorldTransform().cpy().inv().mul(anchor);
        btGeneric6DofSpring2Constraint constraint =
                new btGeneric6DofSpring2Constraint(a.body, b.body, frameA, frameB);
        constraint.setLinearLowerLimit(new Vector3());
        constraint.setLinearUpperLimit(new Vector3());
        constraint.setAngularLowerLimit(vector(settings.angularLowerLimit()));
        constraint.setAngularUpperLimit(vector(settings.angularUpperLimit()));
        if (settings.stiffness() > 0) {
            for (int axis = 3; axis < 6; axis++) {
                constraint.enableSpring(axis, true);
                constraint.setStiffness(axis, (float) settings.stiffness(), true);
                constraint.setDamping(axis, (float) settings.damping(), true);
            }
            constraint.setEquilibriumPoint();
        }
        world.addConstraint(constraint, true);
        constraints.add(constraint);
    }

    @Override
    public SweepResult sweepKinematic(BodyHandle handle, Transform fromBoneTransform,
                                      Transform toBoneTransform) {
        requireOpen();
        BodyEntry entry = entry(handle);
        if (entry.motionType != MotionType.KINEMATIC) {
            throw new IllegalArgumentException("only kinematic bodies are swept");
        }
        Matrix4 fromBody = matrix(fromBoneTransform).mul(entry.boneToCenterOfMass);
        Matrix4 toBody = matrix(toBoneTransform).mul(entry.boneToCenterOfMass);
        float closestFraction = 1;
        String closestBody = null;
        Vector3 fromPosition = new Vector3();
        Vector3 toPosition = new Vector3();

        for (int i = 0; i < entry.childShapes.size(); i++) {
            Matrix4 childLocal = entry.compound.getChildTransform(i).cpy();
            Matrix4 fromChild = fromBody.cpy().mul(childLocal);
            Matrix4 toChild = toBody.cpy().mul(childLocal);
            fromChild.getTranslation(fromPosition);
            toChild.getTranslation(toPosition);
            ClosestNotMeConvexResultCallback callback =
                    new ClosestNotMeConvexResultCallback(
                            entry.body, fromPosition, toPosition);
            try {
                world.convexSweepTest(entry.childShapes.get(i),
                        fromChild, toChild, callback);
                if (callback.hasHit()
                        && callback.getClosestHitFraction() < closestFraction) {
                    closestFraction = callback.getClosestHitFraction();
                    btCollisionObject hit = callback.getHitCollisionObject();
                    closestBody = hit == null ? null : bodyNames.get(hit.getUserValue());
                }
            } finally {
                callback.dispose();
            }
        }
        return closestFraction < 1
                ? new SweepResult(true, closestFraction, closestBody)
                : SweepResult.miss();
    }

    @Override
    public void setKinematicTransform(BodyHandle handle, Transform boneTransform,
                                      double dt, boolean continuousHistory) {
        requireOpen();
        if (!Double.isFinite(dt) || !(dt > 0)) {
            throw new IllegalArgumentException("dt must be finite and greater than zero");
        }
        BodyEntry entry = entry(handle);
        if (entry.motionType != MotionType.KINEMATIC) {
            throw new IllegalArgumentException("body is not kinematic: " + handle.name());
        }

        Matrix4 previousBone = continuousHistory
                ? entry.lastBoneTransform.cpy() : matrix(boneTransform);
        Matrix4 previousBody = previousBone.cpy().mul(entry.boneToCenterOfMass);
        Matrix4 nextBone = matrix(boneTransform);
        Matrix4 nextBody = nextBone.cpy().mul(entry.boneToCenterOfMass);

        Vector3 previousPosition = previousBody.getTranslation(new Vector3());
        Vector3 nextPosition = nextBody.getTranslation(new Vector3());
        Vector3 linearVelocity = nextPosition.cpy().sub(previousPosition)
                .scl((float) (1.0 / dt));
        Vector3 angularVelocity = angularVelocity(previousBody, nextBody, dt);
        if (!continuousHistory) {
            linearVelocity.setZero();
            angularVelocity.setZero();
        }

        entry.body.setInterpolationWorldTransform(previousBody);
        entry.body.setWorldTransform(nextBody);
        entry.body.setLinearVelocity(linearVelocity);
        entry.body.setAngularVelocity(angularVelocity);
        entry.body.setInterpolationLinearVelocity(linearVelocity);
        entry.body.setInterpolationAngularVelocity(angularVelocity);
        entry.body.activate(true);
        entry.lastBoneTransform.set(nextBone);
        world.updateSingleAabb(entry.body);
    }

    @Override
    public void applyCentralForce(BodyHandle handle, double[] force) {
        requireOpen();
        BodyEntry entry = entry(handle);
        if (entry.motionType != MotionType.DYNAMIC) {
            throw new IllegalArgumentException(
                    "central force requires a dynamic body: " + handle.name());
        }
        if (force == null || force.length < 3
                || !Double.isFinite(force[0]) || !Double.isFinite(force[1])
                || !Double.isFinite(force[2])) {
            throw new IllegalArgumentException("force needs three finite components");
        }
        entry.body.applyCentralForce(vector(force));
        entry.body.activate(true);
    }

    @Override
    public void step(double fixedDt) {
        requireOpen();
        if (!Double.isFinite(fixedDt) || !(fixedDt > 0)) {
            throw new IllegalArgumentException(
                    "fixed dt must be finite and greater than zero");
        }
        Map<Integer, MotionSnapshot> before = Map.of();
        if (snapshotLevel != SnapshotLevel.NONE) {
            long started = profiler.start(BakeProfiler.Stage.MOTION_SNAPSHOT_BEFORE);
            before = captureMotionSnapshots();
            profiler.stop(BakeProfiler.Stage.MOTION_SNAPSHOT_BEFORE, started);
        }
        long worldStarted = profiler.start(BakeProfiler.Stage.WORLD_STEP);
        world.stepSimulation((float) fixedDt, 0, (float) fixedDt);
        profiler.stop(BakeProfiler.Stage.WORLD_STEP, worldStarted);
        if (snapshotLevel == SnapshotLevel.NONE) {
            contactSnapshots = List.of();
            return;
        }
        long afterStarted = profiler.start(BakeProfiler.Stage.MOTION_SNAPSHOT_AFTER);
        Map<Integer, MotionSnapshot> after = captureMotionSnapshots();
        profiler.stop(BakeProfiler.Stage.MOTION_SNAPSHOT_AFTER, afterStarted);
        long contactStarted = profiler.start(BakeProfiler.Stage.CONTACT_SNAPSHOT);
        contactSnapshots = captureContactSnapshots(before, after);
        profiler.stop(BakeProfiler.Stage.CONTACT_SNAPSHOT, contactStarted);
        profiler.addCounter(BakeProfiler.Counter.CONTACT_COUNT, contactSnapshots.size());
    }

    @Override
    public BodyState getBodyState(BodyHandle handle) {
        requireOpen();
        BodyEntry entry = entry(handle);
        Matrix4 boneTransform = entry.body.getWorldTransform().cpy()
                .mul(entry.centerOfMassToBone);
        Vector3 linear = entry.body.getLinearVelocity();
        double linearX = linear.x;
        double linearY = linear.y;
        double linearZ = linear.z;
        Vector3 angular = entry.body.getAngularVelocity();
        return new BodyState(transform(boneTransform),
                new double[]{linearX, linearY, linearZ},
                new double[]{angular.x, angular.y, angular.z});
    }

    @Override
    public int getContactCount() {
        requireOpen();
        return contactSnapshots.size();
    }

    @Override
    public double getMaximumPenetration() {
        requireOpen();
        double maximum = 0;
        for (ContactSnapshot contact : contactSnapshots) {
            maximum = Math.max(maximum, contact.penetration());
        }
        return maximum;
    }

    @Override
    public List<ContactSnapshot> getContactSnapshots() {
        requireOpen();
        return contactSnapshots;
    }

    @Override
    public List<BodyShapeSnapshot> getBodyShapeSnapshots() {
        requireOpen();
        if (snapshotLevel != SnapshotLevel.FULL_DIAGNOSTICS) return List.of();
        long started = profiler.start(BakeProfiler.Stage.SHAPE_SNAPSHOT);
        List<BodyShapeSnapshot> result = new ArrayList<>();
        for (Map.Entry<Integer, BodyEntry> bodyEntry : bodies.entrySet()) {
            int id = bodyEntry.getKey();
            BodyEntry entry = bodyEntry.getValue();
            BodyHandle handle = new BodyHandle(id, bodyNames.get(id));
            Matrix4 bodyWorld = entry.body.getWorldTransform().cpy();
            for (int shapeIndex = 0; shapeIndex < entry.childShapes.size(); shapeIndex++) {
                Matrix4 shapeWorld = bodyWorld.cpy()
                        .mul(entry.compound.getChildTransform(shapeIndex));
                result.add(new BodyShapeSnapshot(handle, shapeIndex,
                        entry.halfExtents.get(shapeIndex), transform(shapeWorld)));
            }
        }
        List<BodyShapeSnapshot> snapshots = Collections.unmodifiableList(result);
        profiler.stop(BakeProfiler.Stage.SHAPE_SNAPSHOT, started);
        return snapshots;
    }

    @Override
    public List<NamedBodyState> getBodyStateSnapshots() {
        requireOpen();
        if (snapshotLevel != SnapshotLevel.FULL_DIAGNOSTICS) return List.of();
        long started = profiler.start(BakeProfiler.Stage.STATE_SNAPSHOT);
        List<NamedBodyState> result = new ArrayList<>(bodies.size());
        for (Map.Entry<Integer, BodyEntry> mapEntry : bodies.entrySet()) {
            int id = mapEntry.getKey();
            BodyHandle handle = new BodyHandle(id, bodyNames.get(id));
            MotionSnapshot snapshot = captureMotionSnapshot(mapEntry.getValue());
            result.add(new NamedBodyState(handle, snapshot.state));
        }
        List<NamedBodyState> snapshots = Collections.unmodifiableList(result);
        profiler.stop(BakeProfiler.Stage.STATE_SNAPSHOT, started);
        return snapshots;
    }

    @Override
    public int getNativeBulletVersion() {
        requireOpen();
        return Bullet.VERSION;
    }

    @Override
    public void close() {
        if (closed) return;
        closed = true;
        for (int i = constraints.size() - 1; i >= 0; i--) {
            btGeneric6DofSpring2Constraint constraint = constraints.get(i);
            world.removeConstraint(constraint);
            constraint.dispose();
        }
        constraints.clear();
        List<BodyEntry> entries = new ArrayList<>(bodies.values());
        for (int i = entries.size() - 1; i >= 0; i--) {
            BodyEntry entry = entries.get(i);
            world.removeRigidBody(entry.body);
            entry.body.dispose();
            entry.collisionShape.dispose();
            for (btBoxShape childShape : entry.childShapes) childShape.dispose();
        }
        bodies.clear();
        bodyNames.clear();
        world.dispose();
        solver.dispose();
        broadphase.dispose();
        dispatcher.dispose();
        collisionConfiguration.dispose();
    }

    private BodyEntry entry(BodyHandle handle) {
        Objects.requireNonNull(handle, "handle");
        BodyEntry entry = bodies.get(handle.id());
        if (entry == null || !handle.name().equals(bodyNames.get(handle.id()))) {
            throw new IllegalArgumentException("unknown body handle: " + handle);
        }
        return entry;
    }

    private Map<Integer, MotionSnapshot> captureMotionSnapshots() {
        Map<Integer, MotionSnapshot> snapshots = new HashMap<>();
        for (Map.Entry<Integer, BodyEntry> mapEntry : bodies.entrySet()) {
            snapshots.put(mapEntry.getKey(), captureMotionSnapshot(mapEntry.getValue()));
        }
        return snapshots;
    }

    private static MotionSnapshot captureMotionSnapshot(BodyEntry entry) {
        Matrix4 centerTransform = entry.body.getWorldTransform().cpy();
        Matrix4 boneTransform = centerTransform.cpy().mul(entry.centerOfMassToBone);
        Vector3 linear = entry.body.getLinearVelocity();
        double linearX = linear.x;
        double linearY = linear.y;
        double linearZ = linear.z;
        Vector3 angular = entry.body.getAngularVelocity();
        BodyState state = new BodyState(transform(boneTransform),
                new double[]{linearX, linearY, linearZ},
                new double[]{angular.x, angular.y, angular.z});
        Vector3 center = centerTransform.getTranslation(new Vector3());
        return new MotionSnapshot(state, new double[]{center.x, center.y, center.z});
    }

    private List<ContactSnapshot> captureContactSnapshots(
            Map<Integer, MotionSnapshot> before,
            Map<Integer, MotionSnapshot> after) {
        List<ContactSnapshot> result = new ArrayList<>();
        Vector3 pointA = new Vector3();
        Vector3 pointB = new Vector3();
        Vector3 normal = new Vector3();
        for (int manifoldIndex = 0;
             manifoldIndex < dispatcher.getNumManifolds(); manifoldIndex++) {
            btPersistentManifold manifold =
                    dispatcher.getManifoldByIndexInternal(manifoldIndex);
            int idA = manifold.getBody0().getUserValue();
            int idB = manifold.getBody1().getUserValue();
            MotionSnapshot beforeA = before.get(idA);
            MotionSnapshot beforeB = before.get(idB);
            MotionSnapshot afterA = after.get(idA);
            MotionSnapshot afterB = after.get(idB);
            if (beforeA == null || beforeB == null || afterA == null || afterB == null) {
                continue;
            }
            BodyHandle handleA = new BodyHandle(idA, bodyNames.get(idA));
            BodyHandle handleB = new BodyHandle(idB, bodyNames.get(idB));
            for (int contactIndex = 0;
                 contactIndex < manifold.getNumContacts(); contactIndex++) {
                btManifoldPoint point = manifold.getContactPoint(contactIndex);
                if (point.getDistance() > 0) continue;
                point.getPositionWorldOnA(pointA);
                point.getPositionWorldOnB(pointB);
                point.getNormalWorldOnB(normal);
                double[] pointAArray = vectorArray(pointA);
                double[] pointBArray = vectorArray(pointB);
                double[] normalArray = vectorArray(normal);
                RelativeVelocity relativeBefore = relativeVelocityAtContact(
                        beforeA, beforeB, pointAArray, pointBArray, normalArray);
                RelativeVelocity relativeAfter = relativeVelocityAtContact(
                        afterA, afterB, pointAArray, pointBArray, normalArray);
                result.add(new ContactSnapshot(handleA, handleB,
                        point.getPartId0(), point.getIndex0(),
                        point.getPartId1(), point.getIndex1(),
                        pointAArray, pointBArray, normalArray,
                        Math.max(0, -point.getDistance()), point.getLifeTime(),
                        point.getAppliedImpulse(), relativeBefore.normal,
                        relativeBefore.tangent, relativeAfter.normal,
                        relativeAfter.tangent, beforeA.state, afterA.state,
                        beforeB.state, afterB.state));
            }
        }
        return Collections.unmodifiableList(result);
    }

    private static RelativeVelocity relativeVelocityAtContact(
            MotionSnapshot a, MotionSnapshot b,
            double[] pointA, double[] pointB, double[] normal) {
        double[] velocityA = pointVelocity(a, pointA);
        double[] velocityB = pointVelocity(b, pointB);
        double[] relative = new double[]{
                velocityA[0] - velocityB[0],
                velocityA[1] - velocityB[1],
                velocityA[2] - velocityB[2]
        };
        double normalVelocity = relative[0] * normal[0]
                + relative[1] * normal[1] + relative[2] * normal[2];
        double[] tangent = new double[]{
                relative[0] - normalVelocity * normal[0],
                relative[1] - normalVelocity * normal[1],
                relative[2] - normalVelocity * normal[2]
        };
        return new RelativeVelocity(normalVelocity, tangent);
    }

    private static double[] pointVelocity(MotionSnapshot motion, double[] point) {
        double[] linear = motion.state.linearVelocity();
        double[] angular = motion.state.angularVelocity();
        double rx = point[0] - motion.centerOfMass[0];
        double ry = point[1] - motion.centerOfMass[1];
        double rz = point[2] - motion.centerOfMass[2];
        return new double[]{
                linear[0] + angular[1] * rz - angular[2] * ry,
                linear[1] + angular[2] * rx - angular[0] * rz,
                linear[2] + angular[0] * ry - angular[1] * rx
        };
    }

    private static double[] vectorArray(Vector3 value) {
        return new double[]{value.x, value.y, value.z};
    }

    private void requireOpen() {
        if (closed) throw new IllegalStateException("rigid-body backend is closed");
    }

    private static Matrix4 matrix(Transform transform) {
        double[] translation = transform.translation();
        double[] rotation = transform.rotation();
        return new Matrix4(vector(translation),
                new Quaternion((float) rotation[0], (float) rotation[1],
                        (float) rotation[2], (float) rotation[3]),
                new Vector3(1, 1, 1));
    }

    private static Transform transform(Matrix4 matrix) {
        Vector3 translation = matrix.getTranslation(new Vector3());
        Quaternion rotation = matrix.getRotation(new Quaternion(), true).nor();
        return new Transform(new double[]{translation.x, translation.y, translation.z},
                new double[]{rotation.x, rotation.y, rotation.z, rotation.w});
    }

    private static Vector3 angularVelocity(Matrix4 from, Matrix4 to, double dt) {
        Quaternion previous = from.getRotation(new Quaternion(), true).nor();
        Quaternion next = to.getRotation(new Quaternion(), true).nor();
        Quaternion delta = next.cpy().mul(previous.cpy().conjugate()).nor();
        if (delta.w < 0) delta.mul(-1);
        float clampedW = Math.max(-1, Math.min(1, delta.w));
        double angle = 2.0 * Math.acos(clampedW);
        double sinHalf = Math.sqrt(Math.max(0, 1.0 - clampedW * clampedW));
        if (sinHalf < 1e-7 || angle < 1e-7) return new Vector3();
        return new Vector3((float) (delta.x / sinHalf * angle / dt),
                (float) (delta.y / sinHalf * angle / dt),
                (float) (delta.z / sinHalf * angle / dt));
    }

    private static Vector3 vector(double x, double y, double z) {
        return new Vector3((float) x, (float) y, (float) z);
    }

    private static Vector3 vector(double[] value) {
        return vector(value[0], value[1], value[2]);
    }

    private static final class BodyEntry {
        private final MotionType motionType;
        private final btRigidBody body;
        private final btCollisionShape collisionShape;
        private final btCompoundShape compound;
        private final List<btBoxShape> childShapes;
        private final List<double[]> halfExtents;
        private final Matrix4 boneToCenterOfMass;
        private final Matrix4 centerOfMassToBone;
        private final Matrix4 lastBoneTransform;

        private BodyEntry(MotionType motionType, btRigidBody body,
                          btCollisionShape collisionShape,
                          btCompoundShape compound, List<btBoxShape> childShapes,
                          List<double[]> halfExtents,
                          Matrix4 boneToCenterOfMass, Matrix4 centerOfMassToBone,
                          Matrix4 lastBoneTransform) {
            this.motionType = motionType;
            this.body = body;
            this.collisionShape = collisionShape;
            this.compound = compound;
            this.childShapes = childShapes;
            this.halfExtents = halfExtents;
            this.boneToCenterOfMass = boneToCenterOfMass;
            this.centerOfMassToBone = centerOfMassToBone;
            this.lastBoneTransform = lastBoneTransform;
        }
    }

    private record MotionSnapshot(BodyState state, double[] centerOfMass) {
    }

    private record RelativeVelocity(double normal, double[] tangent) {
    }
}
