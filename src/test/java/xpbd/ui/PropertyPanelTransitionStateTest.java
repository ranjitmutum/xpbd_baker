package xpbd.ui;

import javafx.application.Platform;
import javafx.geometry.Point3D;
import javafx.scene.Group;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.layout.GridPane;
import javafx.scene.shape.Box;
import javafx.scene.shape.Cylinder;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import xpbd.baker.BoneMapper;
import xpbd.loader.BedrockAnimationData;
import xpbd.loader.BedrockModelData;
import xpbd.render.SkeletonView3D;

import java.util.List;
import java.util.Map;
import java.util.concurrent.Callable;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import static org.junit.jupiter.api.Assertions.*;

final class PropertyPanelTransitionStateTest {
    @BeforeAll
    static void startJavaFx() throws Exception {
        CountDownLatch started = new CountDownLatch(1);
        try {
            Platform.startup(started::countDown);
        } catch (IllegalStateException alreadyStarted) {
            started.countDown();
        }
        assertTrue(started.await(10, TimeUnit.SECONDS));
    }

    @AfterAll
    static void stopJavaFx() {
        Platform.exit();
    }

    @Test
    void customModeShowsFieldsClampsBoundsAndNotifies() throws Exception {
        runOnFxThread(() -> {
            PropertyPanel panel = new PropertyPanel();
            panel.setSourceAnimationLength(2.0);
            panel.setAvailableAnimations(Map.of("short", animation(1.0)));
            AtomicInteger changes = new AtomicInteger();
            panel.setOnConfigChanged(changes::incrementAndGet);

            GridPane advanced = (GridPane) panel.lookup("#transitionAdvancedFields");
            ComboBox<String> mode = combo(panel, "#transitionMode");
            ComboBox<String> target = combo(panel, "#transitionTargetAnimation");
            Spinner<Double> sourceExit = spinner(panel, "#transitionSourceExit");
            Spinner<Double> targetEntry = spinner(panel, "#transitionTargetEntry");
            Spinner<Double> duration = spinner(panel, "#transitionDuration");

            assertFalse(advanced.isVisible());
            assertFalse(advanced.isManaged());
            assertEquals(2.0, maximum(sourceExit), 1e-12);
            assertEquals(2.0, sourceExit.getValue(), 1e-12);

            mode.getSelectionModel().select(1);
            assertTrue(advanced.isVisible());
            assertTrue(advanced.isManaged());
            assertTrue(minimum(duration) > 0);

            targetEntry.getValueFactory().setValue(1.8);
            target.setValue("short");
            assertEquals(1.0, maximum(targetEntry), 1e-12);
            assertEquals(1.0, targetEntry.getValue(), 1e-12,
                    "changing to a shorter target must clamp the old value");
            assertTrue(changes.get() >= 3,
                    "mode, time and target changes must invalidate the bake");
            return null;
        });
    }

    @Test
    void selectedBoneWeightCanBeOverriddenAndRemoved() throws Exception {
        runOnFxThread(() -> {
            BedrockModelData.Bone tail = new BedrockModelData.Bone();
            tail.name = "tail";
            PropertyPanel panel = new PropertyPanel();
            panel.setBoneMapper(new BoneMapper(List.of(tail)));
            panel.selectBone(tail.name);
            combo(panel, "#transitionMode").getSelectionModel().select(1);

            CheckBox override = (CheckBox) panel.lookup("#transitionFollowOverride");
            Spinner<Double> weight = spinner(panel, "#transitionFollowWeight");
            assertFalse(override.isSelected());
            assertTrue(weight.isDisabled());

            override.fire();
            weight.getValueFactory().setValue(0.5);
            PropertyPanel.TransitionUiSettings overridden =
                    panel.getTransitionUiSettings();
            assertEquals(0.5, overridden.perBoneFollowWeight().get("tail"), 1e-12);
            assertThrows(UnsupportedOperationException.class,
                    () -> overridden.perBoneFollowWeight().put("tail", 1.0));

            override.fire();
            assertFalse(panel.getTransitionUiSettings()
                    .perBoneFollowWeight().containsKey("tail"));
            return null;
        });
    }

    @Test
    void modelSwitchRemovesWeightsForMissingBones() throws Exception {
        runOnFxThread(() -> {
            BedrockModelData.Bone oldBone = new BedrockModelData.Bone();
            oldBone.name = "old_tail";
            PropertyPanel panel = new PropertyPanel();
            panel.setBoneMapper(new BoneMapper(List.of(oldBone)));
            panel.selectBone(oldBone.name);
            combo(panel, "#transitionMode").getSelectionModel().select(1);
            CheckBox override = (CheckBox) panel.lookup("#transitionFollowOverride");
            override.fire();
            spinner(panel, "#transitionFollowWeight")
                    .getValueFactory().setValue(0.25);
            assertTrue(panel.getTransitionUiSettings()
                    .perBoneFollowWeight().containsKey(oldBone.name));

            BedrockModelData.Bone newBone = new BedrockModelData.Bone();
            newBone.name = "new_tail";
            panel.setBoneMapper(new BoneMapper(List.of(newBone)));

            assertFalse(panel.getTransitionUiSettings()
                    .perBoneFollowWeight().containsKey(oldBone.name));
            return null;
        });
    }

    @Test
    void groundCollisionIsOptInAndRoundTripsThroughConfig() throws Exception {
        runOnFxThread(() -> {
            BedrockModelData.Bone bone = new BedrockModelData.Bone();
            bone.name = "root";
            BoneMapper mapper = new BoneMapper(List.of(bone));
            PropertyPanel panel = new PropertyPanel();
            panel.setBoneMapper(mapper);

            CheckBox ground = (CheckBox) panel.lookup("#groundCollision");
            assertNotNull(ground);
            assertFalse(ground.isSelected());
            ground.fire();
            panel.applyToConfig();

            assertTrue(mapper.getConfig().enableGroundCollision);
            return null;
        });
    }

    @Test
    void realGravityFieldIsOptInAndRoundTripsThroughConfig() throws Exception {
        runOnFxThread(() -> {
            BedrockModelData.Bone bone = new BedrockModelData.Bone();
            bone.name = "root";
            BoneMapper mapper = new BoneMapper(List.of(bone));
            PropertyPanel panel = new PropertyPanel();
            panel.setBoneMapper(mapper);

            CheckBox realGravity = (CheckBox) panel.lookup("#realGravityField");
            assertNotNull(realGravity);
            assertFalse(realGravity.isSelected());
            realGravity.fire();
            panel.applyToConfig();

            assertTrue(mapper.getConfig().enableRealGravityField);
            assertEquals(0, mapper.getEffectiveAnimPullCompliance(bone.name), 0);
            return null;
        });
    }

    @Test
    void viewportGroundAndGridShareTheHorizontalYZeroPlane() throws Exception {
        runOnFxThread(() -> {
            SkeletonView3D view = new SkeletonView3D(320, 240);
            Group root = (Group) view.getSubScene().getRoot();
            Group sceneContent = (Group) root.getChildren().get(0);
            Group environment = (Group) sceneContent.getChildren().get(0);

            Box ground = (Box) environment.getChildren().get(0);
            assertEquals("groundPlane", ground.getId());
            assertEquals(0, ground.getTranslateY() + ground.getHeight() / 2, 1e-12);
            assertEquals(43, environment.getChildren().size());
            for (int index = 1; index < environment.getChildren().size(); index++) {
                Cylinder line = (Cylinder) environment.getChildren().get(index);
                Point3D first = line.localToParent(0, -line.getHeight() / 2, 0);
                Point3D second = line.localToParent(0, line.getHeight() / 2, 0);
                assertEquals(0.01, first.getY(), 1e-9);
                assertEquals(0.01, second.getY(), 1e-9);
                assertEquals(20, first.distance(second), 1e-9);
            }
            return null;
        });
    }

    private static BedrockAnimationData.Animation animation(double length) {
        BedrockAnimationData.Animation animation =
                new BedrockAnimationData.Animation();
        animation.animationLength = length;
        return animation;
    }

    @SuppressWarnings("unchecked")
    private static ComboBox<String> combo(PropertyPanel panel, String selector) {
        return (ComboBox<String>) panel.lookup(selector);
    }

    @SuppressWarnings("unchecked")
    private static Spinner<Double> spinner(PropertyPanel panel, String selector) {
        return (Spinner<Double>) panel.lookup(selector);
    }

    private static double maximum(Spinner<Double> spinner) {
        return ((SpinnerValueFactory.DoubleSpinnerValueFactory)
                spinner.getValueFactory()).getMax();
    }

    private static double minimum(Spinner<Double> spinner) {
        return ((SpinnerValueFactory.DoubleSpinnerValueFactory)
                spinner.getValueFactory()).getMin();
    }

    private static <T> T runOnFxThread(Callable<T> callable) throws Exception {
        AtomicReference<T> result = new AtomicReference<>();
        AtomicReference<Throwable> failure = new AtomicReference<>();
        CountDownLatch finished = new CountDownLatch(1);
        Platform.runLater(() -> {
            try {
                result.set(callable.call());
            } catch (Throwable error) {
                failure.set(error);
            } finally {
                finished.countDown();
            }
        });
        assertTrue(finished.await(10, TimeUnit.SECONDS));
        if (failure.get() instanceof Exception exception) throw exception;
        if (failure.get() instanceof Error error) throw error;
        return result.get();
    }
}
