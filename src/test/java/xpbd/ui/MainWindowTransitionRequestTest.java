package xpbd.ui;

import org.junit.jupiter.api.Test;
import xpbd.baker.TransitionBakeRequest;
import xpbd.loader.BedrockAnimationData;

import java.util.Map;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.*;

final class MainWindowTransitionRequestTest {
    @Test
    void createsCustomRequestWithExplicitTimesAndDefensiveWeights() {
        BedrockAnimationData.Animation source = animation(2.0);
        BedrockAnimationData.Animation target = animation(3.0);
        PropertyPanel.TransitionUiSettings settings =
                new PropertyPanel.TransitionUiSettings(true, "target",
                        0.75, 1.25, 0.4, Map.of("tail", 0.5));

        TransitionBakeRequest request = MainWindow.createTransitionRequest(
                settings, source, Map.of("target", target), Set.of("tail"));

        assertSame(source, request.sourceAnimation());
        assertSame(target, request.targetAnimation());
        assertEquals(0.75, request.sourceExitTime(), 1e-12);
        assertEquals(1.25, request.targetEntryTime(), 1e-12);
        assertEquals(0.4, request.transitionDuration(), 1e-12);
        assertEquals(0.5, request.followWeight("tail"), 1e-12);
        assertThrows(UnsupportedOperationException.class,
                () -> request.perBoneFollowWeight().put("tail", 1.0));
    }

    @Test
    void currentAnimationCanBeBothSourceAndTarget() {
        BedrockAnimationData.Animation source = animation(2.0);
        PropertyPanel.TransitionUiSettings settings =
                new PropertyPanel.TransitionUiSettings(true, null,
                        0.25, 1.0, 0.3, Map.of());

        TransitionBakeRequest request = MainWindow.createTransitionRequest(
                settings, source, Map.of(), Set.of());

        assertSame(source, request.sourceAnimation());
        assertSame(source, request.targetAnimation());
    }

    @Test
    void rejectsInvalidCustomRequestWithUserFacingMessages() {
        BedrockAnimationData.Animation source = animation(1.0);
        BedrockAnimationData.Animation target = animation(1.0);

        IllegalArgumentException sourceTime = assertThrows(
                IllegalArgumentException.class, () -> request(source, target,
                        1.1, 0, 0.2, Map.of(), Set.of()));
        assertTrue(sourceTime.getMessage().contains("源退出时间"));

        IllegalArgumentException targetTime = assertThrows(
                IllegalArgumentException.class, () -> request(source, target,
                        0.5, 1.1, 0.2, Map.of(), Set.of()));
        assertTrue(targetTime.getMessage().contains("目标进入时间"));

        IllegalArgumentException duration = assertThrows(
                IllegalArgumentException.class, () -> request(source, target,
                        0.5, 0, 0, Map.of(), Set.of()));
        assertTrue(duration.getMessage().contains("大于 0"));

        IllegalArgumentException weight = assertThrows(
                IllegalArgumentException.class, () -> request(source, target,
                        0.5, 0, 0.2, Map.of("tail", 1.1), Set.of("tail")));
        assertTrue(weight.getMessage().contains("0 到 1"));

        IllegalArgumentException missingBone = assertThrows(
                IllegalArgumentException.class, () -> request(source, target,
                        0.5, 0, 0.2, Map.of("old_tail", 0.5), Set.of("tail")));
        assertTrue(missingBone.getMessage().contains("不存在的骨骼"));

        PropertyPanel.TransitionUiSettings missingTargetSettings =
                new PropertyPanel.TransitionUiSettings(true, "missing",
                        0.5, 0, 0.2, Map.of());
        IllegalArgumentException missingTarget = assertThrows(
                IllegalArgumentException.class,
                () -> MainWindow.createTransitionRequest(missingTargetSettings,
                        source, Map.of("target", target), Set.of()));
        assertTrue(missingTarget.getMessage().contains("找不到衔接目标动画"));
    }

    private static TransitionBakeRequest request(
            BedrockAnimationData.Animation source,
            BedrockAnimationData.Animation target,
            double sourceExit, double targetEntry, double duration,
            Map<String, Double> weights, Set<String> bones) {
        PropertyPanel.TransitionUiSettings settings =
                new PropertyPanel.TransitionUiSettings(true, "target",
                        sourceExit, targetEntry, duration, weights);
        return MainWindow.createTransitionRequest(
                settings, source, Map.of("target", target), bones);
    }

    private static BedrockAnimationData.Animation animation(double length) {
        BedrockAnimationData.Animation animation =
                new BedrockAnimationData.Animation();
        animation.animationLength = length;
        return animation;
    }
}
