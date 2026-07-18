package xpbd.baker;

import org.junit.jupiter.api.Test;

import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

final class LoopBakeControllerTest {
    @Test
    void requiresConsecutiveStableBoundaryComparisons() {
        AtomicInteger cycle = new AtomicInteger();
        PeriodicStateAdapter adapter = () -> snapshot(cycle.getAndIncrement() == 0
                ? 1.0 : 1.0);
        LoopBakeController controller = new LoopBakeController(
                new LoopBakeConfig(2, 12, 2,
                        1e-6, 1e-6, 1e-6, 1e-6, true), adapter);

        assertFalse(controller.completeCycle().finished());
        assertFalse(controller.completeCycle().finished());
        LoopBakeController.Outcome third = controller.completeCycle();

        assertTrue(third.finished());
        assertTrue(third.converged());
        assertFalse(third.useFallback());
        assertEquals(3, third.completedCycles());
    }

    @Test
    void reportsSeparatePoseVelocityAndContactErrors() {
        PeriodicStateAdapter.Snapshot before = new PeriodicStateAdapter.Snapshot(
                Map.of("tip", new PeriodicStateAdapter.BoneState(
                        new double[]{0, 0, 0}, new double[]{0, 0, 0, 1},
                        new double[]{0, 0, 0}, new double[]{0, 0, 0})),
                Set.of("tip|wall"), 0, 0);
        PeriodicStateAdapter.Snapshot after = new PeriodicStateAdapter.Snapshot(
                Map.of("tip", new PeriodicStateAdapter.BoneState(
                        new double[]{3, 4, 0}, new double[]{0, 0, 1, 0},
                        new double[]{0, 0, 2}, new double[]{0, 3, 4})),
                Set.of(), 0.25, 0);

        LoopErrorReport report = PeriodicStateAdapter.compareSnapshots(before, after);

        assertEquals(5, report.maximumPositionError(), 1e-12);
        assertEquals(Math.PI, report.maximumRotationErrorRadians(), 1e-12);
        assertEquals(2, report.maximumLinearVelocityError(), 1e-12);
        assertEquals(5, report.maximumAngularVelocityError(), 1e-12);
        assertTrue(report.contactSetChanged());
        assertEquals(1, report.contactDifferenceCount());
        assertEquals(0.25, report.maximumPenetration(), 1e-12);
        assertEquals("tip", report.worstBone());
    }

    @Test
    void oneCycleBudgetCanCompareAgainstInitialBoundary() {
        LoopBakeController controller = new LoopBakeController(
                new LoopBakeConfig(1, 1, 1,
                        1e-6, 1e-6, 1e-6, 1e-6, true),
                () -> snapshot(1));

        LoopBakeController.Outcome outcome = controller.completeCycle();

        assertTrue(outcome.finished());
        assertTrue(outcome.converged());
        assertFalse(outcome.useFallback());
        assertEquals(1, outcome.completedCycles());
    }

    @Test
    void contactPenetrationAndAnomaliesBlockConvergence() {
        LoopBakeConfig config = new LoopBakeConfig(1, 2, 1,
                1, 1, 1, 1, 0.2, true);
        LoopErrorReport contactChanged = new LoopErrorReport(
                0, null, 0, null, 0, null, 0, null,
                true, 1, 0, 0);
        LoopErrorReport penetrating = new LoopErrorReport(
                0, null, 0, null, 0, null, 0, null,
                false, 0, 0.21, 0);
        LoopErrorReport anomalous = new LoopErrorReport(
                0, null, 0, null, 0, null, 0, null,
                false, 0, 0, 1);

        assertFalse(contactChanged.isWithin(config));
        assertFalse(penetrating.isWithin(config));
        assertFalse(anomalous.isWithin(config));
    }

    @Test
    void scoresAndWorstBoneAreNormalizedByTheirOwnTolerance() {
        LoopBakeConfig config = new LoopBakeConfig(1, 2, 1,
                10, 0.1, 2, 0.5, 1, true);
        LoopErrorReport report = new LoopErrorReport(
                5, "position", 0.09, "rotation",
                1, "linear", 0.2, "angular",
                false, 0, 0.25, 0);

        assertEquals(0.9, report.normalizedScore(config), 1e-12);
        assertEquals("rotation", report.worstBone(config));
    }

    private static PeriodicStateAdapter.Snapshot snapshot(double x) {
        return new PeriodicStateAdapter.Snapshot(
                Map.of("bone", new PeriodicStateAdapter.BoneState(
                        new double[]{x, 0, 0}, new double[]{0, 0, 0, 1},
                        new double[]{0, 0, 0}, null)), Set.of(), 0, 0);
    }
}
