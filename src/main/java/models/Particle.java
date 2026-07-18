package models;

/** XPBD 模拟粒子，保存位置历史、速度和各类外力影响系数。 */
public final class Particle {
    private final Vector3 position;      // 当前位置
    private final Vector3 prevPosition;  // 上一帧位置（用于速度计算）
    private final Vector3 velocity;      // 当前速度
    private final double invMass;  // 逆质量（0 表示固定粒子）
    private double gravityScale = 1.0;
    private double windInfluence = 1.0;
    private double turbulenceInfluence = 1.0;

    public Particle(double mass) {
        if (!Double.isFinite(mass) || mass < 0) {
            throw new IllegalArgumentException("mass must be zero or a finite positive value");
        }
        double inverse = mass > 0 ? 1.0 / mass : 0.0;
        if (!Double.isFinite(inverse)) {
            throw new IllegalArgumentException("mass is too small to produce a finite inverse mass");
        }
        this.invMass = inverse;
        this.position = new Vector3();
        this.prevPosition = new Vector3();
        this.velocity = new Vector3();
    }

    public Vector3 getPosition() { return position; }
    public void setPosition(Vector3 pos) { position.set(pos); }
    public Vector3 getPrevPosition() { return prevPosition; }
    public void setPrevPosition(Vector3 pos) { prevPosition.set(pos); }
    public Vector3 getVelocity() { return velocity; }
    public void setVelocity(Vector3 vel) { velocity.set(vel); }

    /**
     * 移动固定粒子，并同步维护约束求解所需的位置历史。
     * 该方法将位移解释为本帧运动，因此会更新速度。
     */
    public void setKinematicPosition(Vector3 pos, double dt) {
        if (pos == null || !Double.isFinite(pos.x) || !Double.isFinite(pos.y)
                || !Double.isFinite(pos.z)) {
            throw new IllegalArgumentException("kinematic position must be finite");
        }
        setKinematicPosition(pos.x, pos.y, pos.z, dt);
    }

    public void setKinematicPosition(double x, double y, double z, double dt) {
        if (!isFixed()) {
            throw new IllegalStateException("kinematic positioning requires a fixed particle");
        }
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(z)) {
            throw new IllegalArgumentException("kinematic position must be finite");
        }
        if (!Double.isFinite(dt) || dt <= 0) {
            throw new IllegalArgumentException("dt must be a finite value greater than 0");
        }
        double oldX = position.x;
        double oldY = position.y;
        double oldZ = position.z;
        prevPosition.set(oldX, oldY, oldZ);
        position.set(x, y, z);
        velocity.set((x - oldX) / dt, (y - oldY) / dt, (z - oldZ) / dt);
    }

    /**
     * 同步发生跳变的驱动位置，且不把绝对位移误认为一帧速度。
     * 仅用于明确的动画边界。
     */
    public void synchronizeKinematicPosition(double x, double y, double z) {
        synchronizeKinematicPosition(x, y, z, 0, 0, 0, 1);
    }

    /** 使用显式的周期速度同步边界姿态。 */
    public void synchronizeKinematicPosition(double x, double y, double z,
                                             double vx, double vy, double vz,
                                             double dt) {
        if (!isFixed()) {
            throw new IllegalStateException("kinematic positioning requires a fixed particle");
        }
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(z)
                || !Double.isFinite(vx) || !Double.isFinite(vy)
                || !Double.isFinite(vz) || !Double.isFinite(dt) || !(dt > 0)) {
            throw new IllegalArgumentException("kinematic position must be finite");
        }
        position.set(x, y, z);
        prevPosition.set(x - vx * dt, y - vy * dt, z - vz * dt);
        velocity.set(vx, vy, vz);
    }
    public double getInvMass() { return invMass; }
    public boolean isFixed() { return invMass == 0.0; }
    public double getGravityScale() { return gravityScale; }
    public double getWindInfluence() { return windInfluence; }
    public double getTurbulenceInfluence() { return turbulenceInfluence; }

    public void setForceMultipliers(double gravityScale, double windInfluence,
                                    double turbulenceInfluence) {
        this.gravityScale = finiteNonNegative(gravityScale);
        this.windInfluence = finiteNonNegative(windInfluence);
        this.turbulenceInfluence = finiteNonNegative(turbulenceInfluence);
    }

    private static double finiteNonNegative(double value) {
        return Double.isFinite(value) ? Math.max(0, value) : 0;
    }
}
