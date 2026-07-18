package constraints;

import models.Particle;
import models.Vector3;
import xpbd.baker.BodyColliderCache;

import java.util.Arrays;
import java.util.List;

/**
 * 模拟枢轴粒子与选定运动 Bedrock 立方体面之间的硬单侧碰撞。
 * 本类刻意不保存跨面的持续拉格朗日乘子。
 */
public final class VertexFaceCollisionConstraint implements Constraint {
    private final int[] particleIndices;
    private final BodyColliderCache cache;
    private final List<BodyColliderCache.Collider> colliders;
    private final double skin;
    private final double restitution;
    private final double slop;
    private final double epsilon;
    private final boolean[] touched;
    private final boolean[] fixedReported;
    private final double[] normalX;
    private final double[] normalY;
    private final double[] normalZ;
    private final double[] faceVelocityX;
    private final double[] faceVelocityY;
    private final double[] faceVelocityZ;
    private final double[] desiredRelativeNormal;
    private final double[] localStart = new double[3];
    private final double[] localEnd = new double[3];
    private final double[] localContact = new double[3];
    private final double[] previousWorldContact = new double[3];
    private final double[] currentWorldContact = new double[3];

    private long initialEmbeddedCount;
    private long sweepHitCount;
    private long overlapNoExitCount;
    private long fixedInsideCount;
    private long invalidValueCount;

    // 复用候选状态，避免求解器热路径产生对象分配。

    private int chosenFace;
    private double chosenX;
    private double chosenY;
    private double chosenZ;

    public VertexFaceCollisionConstraint(int[] particleIndices, int particleCount,
                                         BodyColliderCache cache, double skin) {
        this(particleIndices, particleCount, cache, skin, 0.0);
    }

    public VertexFaceCollisionConstraint(int[] particleIndices, int particleCount,
                                         BodyColliderCache cache, double skin,
                                         double restitution) {
        if (cache == null) throw new IllegalArgumentException("collider cache is required");
        if (!Double.isFinite(skin) || skin < 0) {
            throw new IllegalArgumentException("collision skin must be finite and non-negative");
        }
        if (!Double.isFinite(restitution) || restitution < 0 || restitution > 1) {
            throw new IllegalArgumentException(
                    "collision restitution must be between zero and one");
        }
        if (particleCount < 0) throw new IllegalArgumentException("invalid particle count");
        this.particleIndices = particleIndices == null
                ? new int[0] : particleIndices.clone();
        for (int index : this.particleIndices) {
            if (index < 0 || index >= particleCount) {
                throw new IllegalArgumentException("collision particle index is out of range");
            }
        }
        this.cache = cache;
        this.colliders = cache.getColliders();
        this.skin = skin;
        this.restitution = restitution;
        this.epsilon = Math.max(cache.getEpsilon(), 1e-12);
        this.slop = Math.max(epsilon * 8.0, Math.max(1e-7, skin * 1e-5));
        touched = new boolean[particleCount];
        fixedReported = new boolean[particleCount];
        normalX = new double[particleCount];
        normalY = new double[particleCount];
        normalZ = new double[particleCount];
        faceVelocityX = new double[particleCount];
        faceVelocityY = new double[particleCount];
        faceVelocityZ = new double[particleCount];
        desiredRelativeNormal = new double[particleCount];
    }

    @Override
    public void solve(Particle[] particles, double dt) {
        if (colliders.isEmpty()) return;
        for (int item = 0; item < particleIndices.length; item++) {
            int particleIndex = particleIndices[item];
            Particle particle = particles[particleIndex];
            Vector3 position = particle.getPosition();
            Vector3 previous = particle.getPrevPosition();
            if (!finite(position) || !finite(previous)) {
                invalidValueCount++;
                continue;
            }
            if (particle.isFixed()) {
                if (!fixedReported[particleIndex]
                        && cache.containsCurrent(position.x, position.y, position.z, skin)) {
                    fixedInsideCount++;
                    fixedReported[particleIndex] = true;
                }
                continue;
            }
            boolean sweepEligible = cache.isSweepContinuous()
                    && !cache.containsPrevious(previous.x, previous.y, previous.z, skin);
            if (sweepEligible && projectSweep(particle, particleIndex, dt)) {
                continue;
            }
            projectDiscrete(particle, particleIndex, dt, true);
        }
    }

    /** 处理第 0 帧的嵌入问题，且不把投影转换为速度。 */
    public void projectInitial(Particle[] particles) {
        if (colliders.isEmpty()) return;
        int maximumPasses = Math.max(4, colliders.size() * 2);
        for (int item = 0; item < particleIndices.length; item++) {
            int particleIndex = particleIndices[item];
            Particle particle = particles[particleIndex];
            Vector3 position = particle.getPosition();
            if (!finite(position)) {
                invalidValueCount++;
                continue;
            }
            if (!cache.containsCurrent(position.x, position.y, position.z, skin)) continue;
            if (particle.isFixed()) {
                fixedInsideCount++;
                continue;
            }
            initialEmbeddedCount++;
            for (int pass = 0; pass < maximumPasses; pass++) {
                double beforeX = position.x;
                double beforeY = position.y;
                double beforeZ = position.z;
                if (!projectDiscrete(particle, particleIndex, 1.0, false)) break;
                particle.getPrevPosition().x += position.x - beforeX;
                particle.getPrevPosition().y += position.y - beforeY;
                particle.getPrevPosition().z += position.z - beforeZ;
                if (!cache.containsCurrent(position.x, position.y, position.z, skin)) break;
            }
            particle.getVelocity().set(0, 0, 0);
        }
        resetLambda();
    }

    private boolean projectDiscrete(Particle particle, int particleIndex,
                                    double dt, boolean recordVelocity) {
        BodyColliderCache.Collider chosenCollider;
        Vector3 position = particle.getPosition();
        chosenCollider = null;
        double chosenDistanceSquared = Double.POSITIVE_INFINITY;
        int chosenRemainingInside = Integer.MAX_VALUE;
        int containingCount = 0;
        for (int colliderIndex = 0; colliderIndex < colliders.size(); colliderIndex++) {
            BodyColliderCache.Collider collider = colliders.get(colliderIndex);
            if (!collider.containsCurrent(position.x, position.y, position.z, skin)) continue;
            containingCount++;
            for (int face = 0; face < 6; face++) {
                double signedDistance = collider.signedDistanceCurrent(
                        face, position.x, position.y, position.z);
                double correction = skin + slop - signedDistance;
                if (!Double.isFinite(correction) || correction <= 0) continue;
                double nx = collider.getCurrentNormal(face, 0);
                double ny = collider.getCurrentNormal(face, 1);
                double nz = collider.getCurrentNormal(face, 2);
                double targetX = position.x + nx * correction;
                double targetY = position.y + ny * correction;
                double targetZ = position.z + nz * correction;
                int remainingInside = countContainingCurrent(targetX, targetY, targetZ);
                double distanceSquared = correction * correction;
                if (remainingInside < chosenRemainingInside
                        || (remainingInside == chosenRemainingInside
                        && distanceSquared < chosenDistanceSquared)) {
                    chosenCollider = collider;
                    chosenFace = face;
                    chosenX = targetX;
                    chosenY = targetY;
                    chosenZ = targetZ;
                    chosenDistanceSquared = distanceSquared;
                    chosenRemainingInside = remainingInside;
                }
            }
        }
        if (chosenCollider == null) return false;
        if (containingCount > 1 && chosenRemainingInside > 0) overlapNoExitCount++;
        double beforeX = position.x;
        double beforeY = position.y;
        double beforeZ = position.z;
        position.set(chosenX, chosenY, chosenZ);
        if (recordVelocity) {
            recordContact(particle, particleIndex, chosenCollider, chosenFace,
                    beforeX, beforeY, beforeZ, dt);
        }
        return true;
    }

    private int countContainingCurrent(double x, double y, double z) {
        int count = 0;
        for (int i = 0; i < colliders.size(); i++) {
            if (colliders.get(i).containsCurrent(x, y, z, skin)) count++;
        }
        return count;
    }

    private boolean projectSweep(Particle particle, int particleIndex, double dt) {
        Vector3 position = particle.getPosition();
        Vector3 previous = particle.getPrevPosition();
        BodyColliderCache.Collider earliestCollider = null;
        int earliestFace = -1;
        double earliestTime = Double.POSITIVE_INFINITY;
        for (int colliderIndex = 0; colliderIndex < colliders.size(); colliderIndex++) {
            BodyColliderCache.Collider collider = colliders.get(colliderIndex);
            collider.toPreviousBind(previous.x, previous.y, previous.z, localStart);
            collider.toCurrentBind(position.x, position.y, position.z, localEnd);
            double enter = 0;
            double exit = 1;
            int enterFace = -1;
            boolean valid = true;
            for (int face = 0; face < 6; face++) {
                double nx = collider.getBindNormal(face, 0);
                double ny = collider.getBindNormal(face, 1);
                double nz = collider.getBindNormal(face, 2);
                double constant = collider.getBindConstant(face) + skin;
                double startValue = nx * localStart[0] + ny * localStart[1]
                        + nz * localStart[2] - constant;
                double endValue = nx * localEnd[0] + ny * localEnd[1]
                        + nz * localEnd[2] - constant;
                if (startValue <= 0 && endValue <= 0) continue;
                double delta = endValue - startValue;
                if (Math.abs(delta) <= epsilon) {
                    if (startValue > 0) valid = false;
                    if (!valid) break;
                    continue;
                }
                double time = -startValue / delta;
                if (delta < 0) {
                    if (time > enter) {
                        enter = time;
                        enterFace = face;
                    }
                } else {
                    exit = Math.min(exit, time);
                }
                if (enter - exit > epsilon) {
                    valid = false;
                    break;
                }
            }
            if (valid && enterFace >= 0 && enter >= -epsilon && enter <= 1 + epsilon
                    && enter <= exit + epsilon && enter < earliestTime) {
                earliestTime = Math.max(0, enter);
                earliestFace = enterFace;
                earliestCollider = collider;
            }
        }
        if (earliestCollider == null) return false;
        double signed = earliestCollider.signedDistanceCurrent(
                earliestFace, position.x, position.y, position.z);
        double correction = skin + slop - signed;
        if (!Double.isFinite(correction) || correction <= 0) return false;
        double beforeX = position.x;
        double beforeY = position.y;
        double beforeZ = position.z;
        position.x += earliestCollider.getCurrentNormal(earliestFace, 0) * correction;
        position.y += earliestCollider.getCurrentNormal(earliestFace, 1) * correction;
        position.z += earliestCollider.getCurrentNormal(earliestFace, 2) * correction;
        recordContact(particle, particleIndex, earliestCollider, earliestFace,
                beforeX, beforeY, beforeZ, dt);
        sweepHitCount++;
        return true;
    }

    private void recordContact(Particle particle, int particleIndex,
                               BodyColliderCache.Collider collider, int face,
                               double beforeX, double beforeY, double beforeZ,
                               double dt) {
        double nx = collider.getCurrentNormal(face, 0);
        double ny = collider.getCurrentNormal(face, 1);
        double nz = collider.getCurrentNormal(face, 2);
        collider.toCurrentBind(particle.getPosition().x, particle.getPosition().y,
                particle.getPosition().z, localContact);
        collider.fromPreviousBind(localContact[0], localContact[1], localContact[2],
                previousWorldContact);
        collider.fromCurrentBind(localContact[0], localContact[1], localContact[2],
                currentWorldContact);
        double faceX = (currentWorldContact[0] - previousWorldContact[0]) / dt;
        double faceY = (currentWorldContact[1] - previousWorldContact[1]) / dt;
        double faceZ = (currentWorldContact[2] - previousWorldContact[2]) / dt;
        Vector3 previous = particle.getPrevPosition();
        double velocityX = (beforeX - previous.x) / dt;
        double velocityY = (beforeY - previous.y) / dt;
        double velocityZ = (beforeZ - previous.z) / dt;
        double relative = (velocityX - faceX) * nx + (velocityY - faceY) * ny
                + (velocityZ - faceZ) * nz;
        if (!Double.isFinite(relative) || !Double.isFinite(faceX)
                || !Double.isFinite(faceY) || !Double.isFinite(faceZ)) {
            invalidValueCount++;
            return;
        }
        touched[particleIndex] = true;
        normalX[particleIndex] = nx;
        normalY[particleIndex] = ny;
        normalZ[particleIndex] = nz;
        faceVelocityX[particleIndex] = faceX;
        faceVelocityY[particleIndex] = faceY;
        faceVelocityZ[particleIndex] = faceZ;
        desiredRelativeNormal[particleIndex] = relative < 0
                ? -restitution * relative : relative;
    }

    /** 应用配置的法向恢复系数；切向速度保持不变。 */
    public void postSolveVelocity(Particle[] particles) {
        for (int item = 0; item < particleIndices.length; item++) {
            int index = particleIndices[item];
            if (!touched[index]) continue;
            Particle particle = particles[index];
            Vector3 velocity = particle.getVelocity();
            double currentRelative = (velocity.x - faceVelocityX[index]) * normalX[index]
                    + (velocity.y - faceVelocityY[index]) * normalY[index]
                    + (velocity.z - faceVelocityZ[index]) * normalZ[index];
            double correction = desiredRelativeNormal[index] - currentRelative;
            if (!Double.isFinite(correction)) {
                invalidValueCount++;
                continue;
            }
            velocity.x += normalX[index] * correction;
            velocity.y += normalY[index] * correction;
            velocity.z += normalZ[index] * correction;
        }
    }

    @Override
    public void resetLambda() {
        Arrays.fill(touched, false);
        Arrays.fill(fixedReported, false);
    }

    public double getSkin() {
        return skin;
    }

    public Diagnostics getDiagnostics() {
        return new Diagnostics(cache.getDegenerateCubeCount(), initialEmbeddedCount,
                sweepHitCount, overlapNoExitCount, fixedInsideCount,
                invalidValueCount);
    }

    private static boolean finite(Vector3 value) {
        return value != null && Double.isFinite(value.x) && Double.isFinite(value.y)
                && Double.isFinite(value.z);
    }

    public static final class Diagnostics {
        public final int degenerateCubes;
        public final long initialEmbedded;
        public final long sweepHits;
        public final long overlapNoExit;
        public final long fixedInside;
        public final long invalidValues;

        private Diagnostics(int degenerateCubes, long initialEmbedded, long sweepHits,
                            long overlapNoExit, long fixedInside, long invalidValues) {
            this.degenerateCubes = degenerateCubes;
            this.initialEmbedded = initialEmbedded;
            this.sweepHits = sweepHits;
            this.overlapNoExit = overlapNoExit;
            this.fixedInside = fixedInside;
            this.invalidValues = invalidValues;
        }
    }
}
