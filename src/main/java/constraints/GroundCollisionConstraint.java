package constraints;

import models.Particle;
import models.Vector3;

import java.util.Arrays;

/**
 * 与 Y 轴向上的水平地面进行硬单侧碰撞。
 * 动态粒子会保持在 {@code groundY + skin} 之上；固定粒子即使位于平面下方仍由动画驱动。
 */
public final class GroundCollisionConstraint implements Constraint {
    private final int[] particleIndices;
    private final double minimumY;
    private final double restitution;
    private final boolean[] touched;
    private final double[] desiredVelocityY;

    public GroundCollisionConstraint(int[] particleIndices, int particleCount,
                                     double groundY, double skin,
                                     double restitution) {
        if (particleCount < 0) {
            throw new IllegalArgumentException("invalid particle count");
        }
        if (!Double.isFinite(groundY) || !Double.isFinite(skin) || skin < 0) {
            throw new IllegalArgumentException(
                    "ground height and collision skin must be finite and non-negative where applicable");
        }
        if (!Double.isFinite(restitution) || restitution < 0 || restitution > 1) {
            throw new IllegalArgumentException(
                    "ground restitution must be between zero and one");
        }
        this.particleIndices = particleIndices == null
                ? new int[0] : particleIndices.clone();
        for (int index : this.particleIndices) {
            if (index < 0 || index >= particleCount) {
                throw new IllegalArgumentException(
                        "ground particle index is out of range");
            }
        }
        this.minimumY = groundY + skin;
        this.restitution = restitution;
        this.touched = new boolean[particleCount];
        this.desiredVelocityY = new double[particleCount];
    }

    @Override
    public void solve(Particle[] particles, double dt) {
        for (int index : particleIndices) {
            Particle particle = particles[index];
            if (particle.isFixed()) continue;
            Vector3 position = particle.getPosition();
            Vector3 previous = particle.getPrevPosition();
            if (!Double.isFinite(position.y) || !Double.isFinite(previous.y)
                    || position.y >= minimumY) {
                continue;
            }
            if (!touched[index]) {
                double incomingVelocity = (position.y - previous.y) / dt;
                desiredVelocityY[index] = incomingVelocity < 0
                        ? -restitution * incomingVelocity : incomingVelocity;
                touched[index] = true;
            }
            position.y = minimumY;
        }
    }

    /** 投影初始嵌入地面的动态粒子，但不额外生成速度。 */
    public void projectInitial(Particle[] particles) {
        for (int index : particleIndices) {
            Particle particle = particles[index];
            if (particle.isFixed() || particle.getPosition().y >= minimumY) continue;
            double correction = minimumY - particle.getPosition().y;
            particle.getPosition().y += correction;
            particle.getPrevPosition().y += correction;
            particle.getVelocity().y = 0;
        }
        resetLambda();
    }

    /** 在引擎重建速度后施加法向恢复系数。 */
    public void postSolveVelocity(Particle[] particles) {
        for (int index : particleIndices) {
            if (touched[index]) particles[index].getVelocity().y = desiredVelocityY[index];
        }
    }

    public double getMinimumY() {
        return minimumY;
    }

    @Override
    public void resetLambda() {
        Arrays.fill(touched, false);
    }
}
