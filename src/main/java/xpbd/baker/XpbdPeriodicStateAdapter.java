package xpbd.baker;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

/** XPBD 后端的周期状态适配器，封装粒子状态快照、导出骨骼方向和回滚操作。 */
public final class XpbdPeriodicStateAdapter implements PeriodicStateAdapter {
    private final Supplier<PhysicsBaker.BakedFrame> frameSupplier;
    private final Supplier<Double> penetrationSupplier;
    private final Supplier<Integer> anomalySupplier;

    public XpbdPeriodicStateAdapter(Supplier<PhysicsBaker.BakedFrame> frameSupplier,
                                    Supplier<Double> penetrationSupplier,
                                    Supplier<Integer> anomalySupplier) {
        this.frameSupplier = frameSupplier;
        this.penetrationSupplier = penetrationSupplier;
        this.anomalySupplier = anomalySupplier;
    }

    @Override
    public Snapshot capture() {
        PhysicsBaker.BakedFrame frame = frameSupplier.get();
        Map<String, BoneState> bones = new LinkedHashMap<>();
        for (PhysicsBaker.BoneState state : frame.boneStates) {
            if (state.worldPosition == null || state.linearVelocity == null) continue;
            double[] rotation = state.rotation == null ? null
                    : RotationUtil.quaternionFromBedrockEuler(
                    state.rotation[0], state.rotation[1], state.rotation[2]);
            bones.put(state.boneName, new BoneState(
                    state.worldPosition, rotation, state.linearVelocity, null));
        }
        return new Snapshot(bones, Set.of(),
                Math.max(0, penetrationSupplier.get()),
                Math.max(0, anomalySupplier.get()));
    }
}
