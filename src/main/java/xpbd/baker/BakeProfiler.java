package xpbd.baker;

import java.util.Collections;
import java.util.EnumMap;
import java.util.Map;

/**
 * 完整烘焙的可选阶段性能分析器。禁用时避免时钟读取和热路径分配；
 * 启用实例供单个烘焙线程使用，并可在任一步骤后取得快照。
 */
public final class BakeProfiler {
    public enum Stage {
        TOTAL_BAKE,
        INITIALIZE,
        OUTER_REFERENCE_POSE,
        RIGID_BODY_ADVANCE,
        SUBSTEP_POSE_SAMPLE,
        KINEMATIC_DRIVE,
        KINEMATIC_CONVERT,
        KINEMATIC_SWEEP,
        KINEMATIC_SET,
        DYNAMIC_FORCE_TOTAL,
        DYNAMIC_STATE_READ,
        DYNAMIC_FORCE_COMPUTE,
        DYNAMIC_FORCE_SUBMIT,
        WORLD_STEP,
        MOTION_SNAPSHOT_BEFORE,
        MOTION_SNAPSHOT_AFTER,
        CONTACT_SNAPSHOT,
        SHAPE_SNAPSHOT,
        STATE_SNAPSHOT,
        CAPTURE_BONE_OUTPUTS,
        TRANSITION_SAMPLE,
        LOOP_CONTROLLER,
        LOOP_FINALIZATION,
        FINAL_COLLISION_AUDIT,
        RESAMPLE,
        BLEND,
        UNWRAP,
        EXPORT
    }

    public enum Counter {
        BONE_COUNT,
        BODY_COUNT,
        DYNAMIC_BODY_COUNT,
        KINEMATIC_BODY_COUNT,
        COMPOUND_BODY_COUNT,
        CONTACT_COUNT,
        SUBSTEP_COUNT,
        OUTPUT_FRAME_COUNT,
        REUSED_ENDPOINT_POSE_COUNT,
        WARMUP_CYCLE_COUNT,
        LOOP_CANDIDATE_COUNT
    }

    public record Timing(long calls, long totalNanos, long maximumNanos) {
        public double totalMillis() {
            return totalNanos / 1_000_000.0;
        }

        public double averageMicros() {
            return calls == 0 ? 0 : totalNanos / (calls * 1_000.0);
        }
    }

    public record Snapshot(Map<Stage, Timing> timings,
                           Map<Counter, Long> counters) {
        public Snapshot {
            timings = Collections.unmodifiableMap(new EnumMap<>(timings));
            counters = Collections.unmodifiableMap(new EnumMap<>(counters));
        }

        public Timing timing(Stage stage) {
            return timings.getOrDefault(stage, new Timing(0, 0, 0));
        }

        public long counter(Counter counter) {
            return counters.getOrDefault(counter, 0L);
        }
    }

    private static final BakeProfiler DISABLED = new BakeProfiler(false);

    private final boolean enabled;
    private final long[] calls = new long[Stage.values().length];
    private final long[] totalNanos = new long[Stage.values().length];
    private final long[] maximumNanos = new long[Stage.values().length];
    private final long[] counters = new long[Counter.values().length];

    private BakeProfiler(boolean enabled) {
        this.enabled = enabled;
    }

    public static BakeProfiler disabled() {
        return DISABLED;
    }

    public static BakeProfiler enabled() {
        return new BakeProfiler(true);
    }

    public boolean isEnabled() {
        return enabled;
    }

    /** 返回计时起始标记；性能分析禁用时返回零。 */
    public long start(Stage stage) {
        return enabled ? System.nanoTime() : 0;
    }

    public void stop(Stage stage, long startedNanos) {
        if (!enabled) return;
        addNanos(stage, Math.max(0, System.nanoTime() - startedNanos));
    }

    /** 记录外部测量的耗时，例如导出器耗时。 */
    public void addNanos(Stage stage, long elapsedNanos) {
        if (!enabled) return;
        int index = stage.ordinal();
        long elapsed = Math.max(0, elapsedNanos);
        calls[index]++;
        totalNanos[index] += elapsed;
        maximumNanos[index] = Math.max(maximumNanos[index], elapsed);
    }

    public void addCounter(Counter counter, long amount) {
        if (enabled) counters[counter.ordinal()] += amount;
    }

    public void setCounter(Counter counter, long value) {
        if (enabled) counters[counter.ordinal()] = value;
    }

    public Snapshot snapshot() {
        EnumMap<Stage, Timing> timingSnapshot = new EnumMap<>(Stage.class);
        for (Stage stage : Stage.values()) {
            int index = stage.ordinal();
            if (calls[index] != 0 || totalNanos[index] != 0) {
                timingSnapshot.put(stage, new Timing(
                        calls[index], totalNanos[index], maximumNanos[index]));
            }
        }
        EnumMap<Counter, Long> counterSnapshot = new EnumMap<>(Counter.class);
        for (Counter counter : Counter.values()) {
            long value = counters[counter.ordinal()];
            if (value != 0) counterSnapshot.put(counter, value);
        }
        return new Snapshot(timingSnapshot, counterSnapshot);
    }

    public void reset() {
        if (!enabled) return;
        java.util.Arrays.fill(calls, 0);
        java.util.Arrays.fill(totalNanos, 0);
        java.util.Arrays.fill(maximumNanos, 0);
        java.util.Arrays.fill(counters, 0);
    }
}
