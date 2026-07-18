package xpbd.baker;

import java.util.Objects;

/** 每完成一个循环便评估一次的自适应不动点控制器。 */
public final class LoopBakeController {
    private final LoopBakeConfig config;
    private final PeriodicStateAdapter adapter;
    private PeriodicStateAdapter.Snapshot previous;
    private LoopErrorReport latestReport;
    private int completedCycles;
    private int stableCycles;
    private boolean converged;

    public LoopBakeController(LoopBakeConfig config, PeriodicStateAdapter adapter) {
        this.config = Objects.requireNonNull(config, "config");
        this.adapter = Objects.requireNonNull(adapter, "adapter");
        previous = adapter.capture();
    }

    public Outcome completeCycle() {
        completedCycles++;
        PeriodicStateAdapter.Snapshot current = adapter.capture();
        if (previous != null) {
            latestReport = adapter.compare(previous, current);
            boolean stable = completedCycles >= config.minimumWarmupCycles()
                    && latestReport.isWithin(config);
            stableCycles = stable ? stableCycles + 1 : 0;
            converged = stableCycles >= config.requiredStableCycles();
        }
        previous = current;
        boolean finished = converged
                || completedCycles >= config.maximumWarmupCycles();
        double score = latestReport == null
                ? Double.POSITIVE_INFINITY : latestReport.normalizedScore(config);
        return new Outcome(completedCycles, stableCycles, latestReport,
                converged, finished, finished && !converged
                && config.seamFallbackEnabled(), score);
    }

    public record Outcome(int completedCycles, int stableCycles,
                          LoopErrorReport report, boolean converged,
                          boolean finished, boolean useFallback,
                          double normalizedScore) {
    }
}
