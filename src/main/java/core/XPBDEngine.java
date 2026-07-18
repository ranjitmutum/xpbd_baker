package core;

import constraints.Constraint;
import models.Particle;
import models.Vector3;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/** XPBD 粒子系统的时间步推进器：负责外力积分、约束投影与速度回算。 */
public final class XPBDEngine {
    private final List<Particle> particles = new ArrayList<>();
    private final List<Constraint> constraints = new ArrayList<>();
    private Particle[] particleCache = new Particle[0];
    private boolean particleCacheDirty = true;
    private Vector3 gravity = new Vector3(0, -9.8, 0);
    private Vector3 windVelocity = new Vector3();
    private double airDrag = 0;
    private double turbulence = 0;
    private double elapsedTime = 0;
    private int solverIterations = 4;   // 每次时间步的迭代次数

    // --- 粒子管理 ---
    public void addParticle(Particle p) {
        particles.add(Objects.requireNonNull(p, "particle"));
        particleCacheDirty = true;
    }

    public Particle getParticle(int index) {
        return particles.get(index);
    }

    // --- 约束管理 ---
    public void addConstraint(Constraint c) {
        constraints.add(Objects.requireNonNull(c, "constraint"));
    }

    // --- 参数设置 ---
    public void setGravity(Vector3 g) {
        Objects.requireNonNull(g, "gravity");
        gravity = finiteVector(g);
    }

    public void setSolverIterations(int iterations) {
        this.solverIterations = Math.max(1, iterations);
    }

    /**
     * @param velocity 模型单位下的目标空气速度（每秒）
     * @param drag 响应速率（每秒）；为 0 时禁用空气阻力
     * @param turbulenceAcceleration 阵风加速度（模型单位/秒²）
     */
    public void setAerodynamics(Vector3 velocity, double drag, double turbulenceAcceleration) {
        windVelocity = velocity != null ? finiteVector(velocity) : new Vector3();
        airDrag = Double.isFinite(drag) ? Math.max(0, drag) : 0;
        turbulence = Double.isFinite(turbulenceAcceleration)
                ? Math.max(0, turbulenceAcceleration) : 0;
    }

    public void clear() {
        particles.clear();
        constraints.clear();
        particleCache = new Particle[0];
        particleCacheDirty = true;
        elapsedTime = 0;
    }

    /**
     * 执行一个完整的时间步
     * @param dt 时间步长（秒），建议 1/60 或更小，更大的时间步长理论上不会造成崩溃，但是会有精度问题
     */
    public void step(double dt) {
        step(dt, elapsedTime + dt, 0);
    }

    /**
     * 使用显式、确定性的外力时钟推进一个时间步。正周期会使每次阵风均为该周期的整数谐波。
     */
    public void step(double dt, double forcingTime, double forcingPeriod) {
        if (!Double.isFinite(dt) || dt <= 0) {
            throw new IllegalArgumentException("dt must be a finite value greater than 0");
        }
        if (!Double.isFinite(forcingTime) || !Double.isFinite(forcingPeriod)
                || forcingPeriod < 0) {
            throw new IllegalArgumentException("forcing time/period must be finite");
        }
        // 粒子增删后才重建数组，避免每个时间步重复分配。
        if (particleCacheDirty) {
            particleCache = particles.toArray(new Particle[0]);
            particleCacheDirty = false;
        }
        Particle[] pArray = particleCache;

        // 重置所有约束的 lambda
        for (Constraint c : constraints) {
            c.resetLambda();
        }

        // 外力和位置预测合并成一轮连续遍历。
        for (int i = 0; i < pArray.length; i++) {
            Particle p = pArray[i];
            if (!p.isFixed()) {
                p.getPrevPosition().set(p.getPosition());
                Vector3 velocity = p.getVelocity();
                double gravityFactor = dt * p.getGravityScale();
                velocity.x += gravity.x * gravityFactor;
                velocity.y += gravity.y * gravityFactor;
                velocity.z += gravity.z * gravityFactor;

                if (airDrag > 0 && p.getWindInfluence() > 0) {
                    // 指数响应在时间步长变化时仍能保持稳定。
                    double response = 1.0 - Math.exp(
                            -airDrag * p.getWindInfluence() * dt);
                    velocity.x += (windVelocity.x - velocity.x) * response;
                    velocity.y += (windVelocity.y - velocity.y) * response;
                    velocity.z += (windVelocity.z - velocity.z) * response;
                }

                if (turbulence > 0 && p.getTurbulenceInfluence() > 0) {
                    // 确定性的相位偏移避免相邻粒子像刚性平面一样运动，同时保证烘焙可复现。
                    double basePhase = forcingPeriod > 0
                            ? 2.0 * Math.PI * wrappedPhase(forcingTime, forcingPeriod)
                            : forcingTime * 3.7;
                    double phase = basePhase + i * 1.61803398875;
                    double gust = turbulence * p.getTurbulenceInfluence();
                    velocity.x += Math.sin(phase) * gust * dt;
                    velocity.y += Math.sin(basePhase * 2.0
                            + i * 0.73 + 1.7) * gust * 0.15 * dt;
                    velocity.z += Math.cos(basePhase * 3.0
                            + i * 1.13 + 0.4) * gust * dt;
                }
                p.getPosition().x += velocity.x * dt;
                p.getPosition().y += velocity.y * dt;
                p.getPosition().z += velocity.z * dt;
            }
        }

        // 迭代求解约束（阻尼已内嵌于约束中）
        for (int iter = 0; iter < solverIterations; iter++) {
            for (Constraint c : constraints) {
                c.solve(pArray, dt);
            }
        }

        // 约束投影后才能回算速度，因此这是每步唯一的第二轮粒子遍历。
        for (Particle p : pArray) {
            if (!p.isFixed()) {
                Vector3 newPos = p.getPosition();
                Vector3 oldPos = p.getPrevPosition();
                p.getVelocity().set(
                        (newPos.x - oldPos.x) / dt,
                        (newPos.y - oldPos.y) / dt,
                        (newPos.z - oldPos.z) / dt
                );
            }
        }
        elapsedTime += dt;
    }

    private static double wrappedPhase(double time, double period) {
        double wrapped = time % period;
        if (wrapped < 0) wrapped += period;
        return wrapped / period;
    }

    private static Vector3 finiteVector(Vector3 value) {
        return new Vector3(
                Double.isFinite(value.x) ? value.x : 0,
                Double.isFinite(value.y) ? value.y : 0,
                Double.isFinite(value.z) ? value.z : 0);
    }
}
