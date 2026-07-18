package xpbd.ui;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

final class ControlBarStateTest {
    @Test
    void staleBakeKeepsExportActionClickableWhileIdle() {
        assertFalse(ControlBar.shouldDisableExport(false));
    }

    @Test
    void backgroundTaskStillDisablesExportAction() {
        assertTrue(ControlBar.shouldDisableExport(true));
    }
}
