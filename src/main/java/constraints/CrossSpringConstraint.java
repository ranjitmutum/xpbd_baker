package constraints;

import models.Particle;

/**
 * 三粒子关节的角度范围近似约束：限制两端粒子之间的距离处于允许区间，
 * 从而以稳定的方式间接限制夹角。
 */
public final class CrossSpringConstraint implements Constraint {
    private static final double EPSILON = 1e-9;
    private final int idxA;
    private final int idxC;
    private final double minDistance;
    private final double maxDistance;
    private final double compliance;
    private final double fallbackX;
    private final double fallbackY;
    private final double fallbackZ;
    private double lambda;
    private int activeSide;

    public CrossSpringConstraint(int idxA, int idxC, double minDistance,
                                 double maxDistance, double compliance,
                                 double fallbackX, double fallbackY,
                                 double fallbackZ) {
        this.idxA = idxA;
        this.idxC = idxC;
        double safeMin = finiteNonNegative(minDistance);
        double safeMax = finiteNonNegative(maxDistance);
        this.minDistance = Math.min(safeMin, safeMax);
        this.maxDistance = Math.max(safeMin, safeMax);
        this.compliance = finiteNonNegative(compliance);
        double length = Math.sqrt(fallbackX * fallbackX + fallbackY * fallbackY
                + fallbackZ * fallbackZ);
        if (Double.isFinite(length) && length > EPSILON) {
            this.fallbackX = fallbackX / length;
            this.fallbackY = fallbackY / length;
            this.fallbackZ = fallbackZ / length;
        } else {
            this.fallbackX = 1;
            this.fallbackY = 0;
            this.fallbackZ = 0;
        }
    }

    @Override
    public void solve(Particle[] particles, double dt) {
        Particle a = particles[idxA];
        Particle c = particles[idxC];
        double dx = c.getPosition().x - a.getPosition().x;
        double dy = c.getPosition().y - a.getPosition().y;
        double dz = c.getPosition().z - a.getPosition().z;
        double distance = Math.sqrt(dx * dx + dy * dy + dz * dz);
        int side;
        double boundary;
        // 仅在距离越过上下界时施加修正；区间内不产生约束力。
        if (distance < minDistance) {
            side = -1;
            boundary = minDistance;
        } else if (distance > maxDistance) {
            side = 1;
            boundary = maxDistance;
        } else {
            lambda = 0;
            activeSide = 0;
            return;
        }
        if (side != activeSide) lambda = 0;
        activeSide = side;

        double nx;
        double ny;
        double nz;
        if (distance > EPSILON) {
            nx = dx / distance;
            ny = dy / distance;
            nz = dz / distance;
        } else {
            nx = fallbackX;
            ny = fallbackY;
            nz = fallbackZ;
        }
        double wa = a.getInvMass();
        double wc = c.getInvMass();
        double alpha = compliance / (dt * dt);
        double denominator = wa + wc + alpha;
        if (!Double.isFinite(denominator) || denominator <= 0) return;
        double value = distance - boundary;
        double deltaLambda = -(value + alpha * lambda) / denominator;
        lambda += deltaLambda;
        a.getPosition().x -= nx * deltaLambda * wa;
        a.getPosition().y -= ny * deltaLambda * wa;
        a.getPosition().z -= nz * deltaLambda * wa;
        c.getPosition().x += nx * deltaLambda * wc;
        c.getPosition().y += ny * deltaLambda * wc;
        c.getPosition().z += nz * deltaLambda * wc;
    }

    @Override
    public void resetLambda() {
        lambda = 0;
        activeSide = 0;
    }

    private static double finiteNonNegative(double value) {
        return Double.isFinite(value) ? Math.max(0, value) : 0;
    }
}
