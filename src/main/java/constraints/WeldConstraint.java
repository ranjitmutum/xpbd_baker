package constraints;

import models.Particle;

import java.util.Arrays;

/**
 * 将两个枢轴焊接到同一位置的三轴 XPBD 约束。
 * 相比零长度的距离约束，该实现不需要归一化方向，因此不会在重合点出现未定义方向。
 */
public final class WeldConstraint implements Constraint {
    private final int idxA;
    private final int idxB;
    private final double compliance;
    private final double dampingCompliance;
    private final double[] lambda = new double[3];

    public WeldConstraint(int idxA, int idxB, double compliance,
                          double dampingCompliance) {
        this.idxA = idxA;
        this.idxB = idxB;
        this.compliance = finiteNonNegative(compliance);
        this.dampingCompliance = finiteNonNegative(dampingCompliance);
    }

    @Override
    public void solve(Particle[] particles, double dt) {
        Particle a = particles[idxA];
        Particle b = particles[idxB];
        double wa = a.getInvMass();
        double wb = b.getInvMass();
        double weight = wa + wb;
        double alpha = compliance / (dt * dt);
        double gamma = dampingCompliance / dt;
        double denominator = (1.0 + gamma) * weight + alpha;
        if (!Double.isFinite(denominator) || denominator <= 0) return;

        solveAxis(a, b, 0, b.getPosition().x - a.getPosition().x,
                implicitVelocityX(b, dt) - implicitVelocityX(a, dt),
                wa, wb, alpha, gamma, dt, denominator);
        solveAxis(a, b, 1, b.getPosition().y - a.getPosition().y,
                implicitVelocityY(b, dt) - implicitVelocityY(a, dt),
                wa, wb, alpha, gamma, dt, denominator);
        solveAxis(a, b, 2, b.getPosition().z - a.getPosition().z,
                implicitVelocityZ(b, dt) - implicitVelocityZ(a, dt),
                wa, wb, alpha, gamma, dt, denominator);
    }

    private void solveAxis(Particle a, Particle b, int axis, double value,
                           double derivative, double wa, double wb,
                           double alpha, double gamma, double dt,
                           double denominator) {
        double deltaLambda = -(value + alpha * lambda[axis]
                + gamma * dt * derivative) / denominator;
        lambda[axis] += deltaLambda;
        if (axis == 0) {
            a.getPosition().x -= wa * deltaLambda;
            b.getPosition().x += wb * deltaLambda;
        } else if (axis == 1) {
            a.getPosition().y -= wa * deltaLambda;
            b.getPosition().y += wb * deltaLambda;
        } else {
            a.getPosition().z -= wa * deltaLambda;
            b.getPosition().z += wb * deltaLambda;
        }
    }

    private static double implicitVelocityX(Particle p, double dt) {
        return (p.getPosition().x - p.getPrevPosition().x) / dt;
    }

    private static double implicitVelocityY(Particle p, double dt) {
        return (p.getPosition().y - p.getPrevPosition().y) / dt;
    }

    private static double implicitVelocityZ(Particle p, double dt) {
        return (p.getPosition().z - p.getPrevPosition().z) / dt;
    }

    @Override
    public void resetLambda() {
        Arrays.fill(lambda, 0);
    }

    private static double finiteNonNegative(double value) {
        return Double.isFinite(value) ? Math.max(0, value) : 0;
    }
}
