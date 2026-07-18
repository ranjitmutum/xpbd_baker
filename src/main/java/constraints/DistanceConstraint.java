package constraints;

import models.Particle;

public final class DistanceConstraint implements Constraint {
    private final int idxA, idxB;
    private final double restLength;
    private final double compliance;        // 位置柔度 (0 = 刚性)
    private final double dampingCompliance; // 阻尼柔度 (瑞利耗散势)
    private double lambda;                  // 累积拉格朗日乘子
    private double normalX = 1.0;
    private double normalY;
    private double normalZ;

    /**
     * @param idxA             粒子 A 索引
     * @param idxB             粒子 B 索引
     * @param restLength       静止长度
     * @param compliance       位置柔度 (建议 0 ~ 1e-3，0 表示完全刚性)
     * @param dampingCompliance 阻尼柔度 (建议为 compliance 的 0.1~1 倍)
     */
    public DistanceConstraint(int idxA, int idxB, double restLength,
                              double compliance, double dampingCompliance) {
        this.idxA = idxA;
        this.idxB = idxB;
        this.restLength = finiteNonNegative(restLength);
        this.compliance = finiteNonNegative(compliance);
        this.dampingCompliance = finiteNonNegative(dampingCompliance);
        this.lambda = 0.0;
    }

    @Override
    public void solve(Particle[] particles, double dt) {
        Particle pA = particles[idxA];
        Particle pB = particles[idxB];
        double invMassA = pA.getInvMass();
        double invMassB = pB.getInvMass();
        if (invMassA == 0 && invMassB == 0) return; // 两个固定点，无意义

        double dx = pB.getPosition().x - pA.getPosition().x;
        double dy = pB.getPosition().y - pA.getPosition().y;
        double dz = pB.getPosition().z - pA.getPosition().z;
        double distSquared = dx * dx + dy * dy + dz * dz;
        if (!Double.isFinite(distSquared)) return;
        double dist = Math.sqrt(distSquared);
        double nx;
        double ny;
        double nz;
        if (dist > 1e-8) {
            nx = dx / dist;
            ny = dy / dist;
            nz = dz / dist;
            normalX = nx;
            normalY = ny;
            normalZ = nz;
        } else if (restLength > 0) {
            nx = normalX;
            ny = normalY;
            nz = normalZ;
        } else {
            return;
        }

        // 约束函数值 C = |pB-pA| - restLength
        double C = dist - restLength;

        // 本轮迭代的隐式速度：前序约束投影应立即影响阻尼；公开速度缓存仅在完整位置求解后更新。
        double velocityX = ((pB.getPosition().x - pB.getPrevPosition().x)
                - (pA.getPosition().x - pA.getPrevPosition().x)) / dt;
        double velocityY = ((pB.getPosition().y - pB.getPrevPosition().y)
                - (pA.getPosition().y - pA.getPrevPosition().y)) / dt;
        double velocityZ = ((pB.getPosition().z - pB.getPrevPosition().z)
                - (pA.getPosition().z - pA.getPrevPosition().z)) / dt;
        double Cdot = nx * velocityX + ny * velocityY + nz * velocityZ;

        // XPBD 带阻尼的公式
        double alphaTilde = compliance / (dt * dt);
        // 把阻尼同时放入有效质量，较大的 UI 数值只会趋向临界阻尼，
        // 不会像旧公式一样生成无上限的位置冲量。
        double gamma = dampingCompliance / dt;
        double invMassSum = invMassA + invMassB;
        double denominator = (1.0 + gamma) * invMassSum + alphaTilde;

        if (!Double.isFinite(denominator) || denominator <= 0) return;

        double deltaLambda = -(C + alphaTilde * lambda + gamma * dt * Cdot) / denominator;

        // 位置修正
        pA.getPosition().x -= nx * deltaLambda * invMassA;
        pA.getPosition().y -= ny * deltaLambda * invMassA;
        pA.getPosition().z -= nz * deltaLambda * invMassA;
        pB.getPosition().x += nx * deltaLambda * invMassB;
        pB.getPosition().y += ny * deltaLambda * invMassB;
        pB.getPosition().z += nz * deltaLambda * invMassB;

        // 更新拉格朗日乘子
        lambda += deltaLambda;
    }

    @Override
    public void resetLambda() {
        lambda = 0.0;
    }

    private static double finiteNonNegative(double value) {
        return Double.isFinite(value) ? Math.max(0, value) : 0;
    }

}
