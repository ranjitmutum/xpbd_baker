package constraints;

import models.Particle;

import java.util.Arrays;

/** 用于动画跟随的三轴 XPBD 目标约束。 */
public final class TargetConstraint implements Constraint {
    private final int particleIndex;
    private final double compliance;
    private final double[] target = new double[3];
    private final double[] lambda = new double[3];

    public TargetConstraint(int particleIndex, double compliance) {
        this.particleIndex = particleIndex;
        this.compliance = Double.isFinite(compliance) ? Math.max(0, compliance) : 0;
    }

    public void setTarget(double x, double y, double z) {
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(z)) {
            throw new IllegalArgumentException("target position must be finite");
        }
        target[0] = x;
        target[1] = y;
        target[2] = z;
    }

    @Override
    public void solve(Particle[] particles, double dt) {
        Particle particle = particles[particleIndex];
        double weight = particle.getInvMass();
        double alpha = compliance / (dt * dt);
        double denominator = weight + alpha;
        if (!Double.isFinite(denominator) || denominator <= 0) return;

        // 三个轴相互独立求解，避免零长度方向向量带来的不稳定性。
        double dx = -(particle.getPosition().x - target[0] + alpha * lambda[0])
                / denominator;
        double dy = -(particle.getPosition().y - target[1] + alpha * lambda[1])
                / denominator;
        double dz = -(particle.getPosition().z - target[2] + alpha * lambda[2])
                / denominator;
        lambda[0] += dx;
        lambda[1] += dy;
        lambda[2] += dz;
        particle.getPosition().x += weight * dx;
        particle.getPosition().y += weight * dy;
        particle.getPosition().z += weight * dz;
    }

    @Override
    public void resetLambda() {
        Arrays.fill(lambda, 0);
    }
}
