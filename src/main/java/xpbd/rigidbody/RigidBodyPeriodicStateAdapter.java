package xpbd.rigidbody;

import xpbd.baker.PeriodicStateAdapter;

import java.util.Objects;

/** 保留角速度和接触数据的 Bullet 专用周期状态捕获适配器。 */
public final class RigidBodyPeriodicStateAdapter implements PeriodicStateAdapter {
    private final RigidBodyBakeSession session;

    public RigidBodyPeriodicStateAdapter(RigidBodyBakeSession session) {
        this.session = Objects.requireNonNull(session, "session");
    }

    @Override
    public Snapshot capture() {
        return session.capturePeriodicSnapshot();
    }
}
