package constraints;

import models.Particle;

/**
 * 所有 XPBD 约束类型的统一接口。
 */
public interface Constraint {
    /**
     * 求解约束，更新粒子位置。
     * @param particles 全局粒子数组
     * @param dt        时间步长（秒）
     */
    void solve(Particle[] particles, double dt);

    void resetLambda();
}
