package xpbd.regression;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import javafx.application.Platform;
import javafx.scene.Scene;
import javafx.scene.image.PixelReader;
import javafx.scene.image.WritableImage;
import javafx.scene.layout.StackPane;
import javafx.stage.Stage;
import org.junit.jupiter.api.Test;
import xpbd.baker.BoneMapper;
import xpbd.baker.PhysicsBaker;
import xpbd.loader.BedrockAnimationData;
import xpbd.loader.BedrockModelData;
import xpbd.loader.ModelLoader;
import xpbd.render.SkeletonView3D;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.Reader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

/** Opt-in visual acceptance capture using the product's real JavaFX 3D renderer. */
final class RealModelGuiEvidenceTest {
    private static final int WIDTH = 1280;
    private static final int HEIGHT = 720;

    @Test
    void renderBakedRunWithProductViewport() throws Exception {
        String modelProperty = System.getProperty("xpbd.test.shiroModel");
        String evidenceProperty = System.getProperty("xpbd.evidenceDir");
        assumeTrue(modelProperty != null && !modelProperty.isBlank(),
                "set -Dxpbd.test.shiroModel to the real model");
        assumeTrue(evidenceProperty != null && !evidenceProperty.isBlank(),
                "set -Dxpbd.evidenceDir to capture GUI evidence");

        Path modelPath = Path.of(modelProperty).toAbsolutePath().normalize();
        Path evidenceDir = Path.of(evidenceProperty).toAbsolutePath().normalize();
        Path animationPath = evidenceDir.resolve("run_120fps.animation.json");
        assumeTrue(Files.isRegularFile(animationPath),
                "run RealModelCollisionEvidenceTest first");
        Path framesDir = evidenceDir.resolve("gui_frames");
        Files.createDirectories(framesDir);

        BedrockModelData.Geometry geometry = ModelLoader.load(modelPath.toString());
        BedrockAnimationData.Animation run;
        try (Reader reader = Files.newBufferedReader(animationPath, StandardCharsets.UTF_8)) {
            JsonObject root = JsonParser.parseReader(reader).getAsJsonObject();
            run = BedrockAnimationData.Animation.fromJson(
                    root.getAsJsonObject("animations").getAsJsonObject("run"));
        }

        BoneMapper mapper = configuredMapper(geometry);
        List<PhysicsBaker.BakedFrame> frames;
        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setSourceAnimation(run);
            baker.setDt(1.0 / 120.0);
            baker.initialize();
            baker.runToEnd();
            assertEquals(1.0 / 120.0, baker.getOutputFrameInterval(), 1e-12);
            assertTrue(baker.getFrames().size() >= 160);
            frames = List.copyOf(baker.getFrames());
        }

        AtomicReference<Throwable> failure = new AtomicReference<>();
        CountDownLatch finished = new CountDownLatch(1);
        Platform.startup(() -> {
            Stage stage = null;
            try {
                SkeletonView3D view = new SkeletonView3D(WIDTH, HEIGHT);
                view.setGeometry(visualRegion(geometry));
                view.setBoneMapper(mapper);
                view.getCameraController().setView(5, 0, 0, -12, -55);
                StackPane root = new StackPane(view.getSubScene());
                stage = new Stage();
                stage.setTitle("XPBD Bone Baker - rigid-body run evidence");
                stage.setScene(new Scene(root, WIDTH, HEIGHT));
                stage.show();
                root.applyCss();
                root.layout();

                for (int index = 0; index < frames.size(); index++) {
                    PhysicsBaker.BakedFrame frame = frames.get(index);
                    view.resetPose();
                    view.applyAnimationPose(run, frame.time);
                    view.applyBakedFrame(frame);
                    root.applyCss();
                    root.layout();
                    WritableImage snapshot = new WritableImage(WIDTH, HEIGHT);
                    view.getSubScene().snapshot(null, snapshot);
                    Path framePath = framesDir.resolve(
                            String.format("frame_%04d.png", index));
                    ImageIO.write(toBufferedImage(snapshot), "png", framePath.toFile());
                    if (index == 0 || index == frames.size() / 2
                            || index == frames.size() - 1) {
                        String label = index == 0 ? "start"
                                : index == frames.size() - 1 ? "end" : "mid";
                        ImageIO.write(toBufferedImage(snapshot), "png",
                                evidenceDir.resolve("gui_keyframe_" + label + ".png").toFile());
                    }
                }
            } catch (Throwable error) {
                failure.set(error);
            } finally {
                if (stage != null) stage.close();
                finished.countDown();
                Platform.exit();
            }
        });
        assertTrue(finished.await(120, TimeUnit.SECONDS),
                "JavaFX evidence capture timed out");
        if (failure.get() != null) throw new AssertionError(failure.get());
    }

    private static BoneMapper configuredMapper(BedrockModelData.Geometry geometry) {
        BoneMapper mapper = new BoneMapper(geometry.bones);
        List.of("Right_Line", "Right_Line2", "Left_Line", "Left_Line2")
                .forEach(mapper::addPhysicsBone);
        mapper.addCollisionRoot("RightLeg");
        mapper.addCollisionRoot("LeftLeg");
        BoneMapper.PhysicsGroupConfig config = mapper.getConfig();
        config.simulationMode = BoneMapper.SimulationMode.RIGID_BODY;
        config.rigidBodySubsteps = 1;
        config.movementSpeed = 4.2;
        config.movementDirectionDegrees = -90;
        config.movementElevationDegrees = 0;
        config.airDrag = 2;
        config.turbulence = 1.5;
        config.solverIterations = 8;
        config.animationPullCompliance = 0.1;
        config.collisionSkin = 0.1;
        config.rigidBodyMaximumSafePenetration = 10;
        return mapper;
    }

    private static BedrockModelData.Geometry visualRegion(
            BedrockModelData.Geometry geometry) {
        Map<String, BedrockModelData.Bone> byName = new HashMap<>();
        for (BedrockModelData.Bone bone : geometry.bones) byName.put(bone.name, bone);
        Set<String> visible = new HashSet<>();
        for (BedrockModelData.Bone bone : geometry.bones) {
            if (isDescendantOf(bone, "BackWaistBow", byName)
                    || isDescendantOf(bone, "RightLeg", byName)
                    || isDescendantOf(bone, "LeftLeg", byName)) {
                visible.add(bone.name);
            }
        }
        Set<String> retained = new HashSet<>(visible);
        for (String name : List.copyOf(visible)) {
            BedrockModelData.Bone bone = byName.get(name);
            while (bone != null && bone.parent != null) {
                retained.add(bone.parent);
                bone = byName.get(bone.parent);
            }
        }

        BedrockModelData.Geometry result = new BedrockModelData.Geometry();
        result.description = geometry.description;
        for (BedrockModelData.Bone source : geometry.bones) {
            if (!retained.contains(source.name)) continue;
            BedrockModelData.Bone copy = new BedrockModelData.Bone();
            copy.name = source.name;
            copy.parent = source.parent;
            copy.pivot = source.pivot.clone();
            copy.rotation = source.rotation.clone();
            copy.cubes = visible.contains(source.name)
                    ? source.cubes : List.of();
            result.bones.add(copy);
        }
        return result;
    }

    private static boolean isDescendantOf(
            BedrockModelData.Bone bone, String root,
            Map<String, BedrockModelData.Bone> byName) {
        BedrockModelData.Bone current = bone;
        while (current != null) {
            if (root.equals(current.name)) return true;
            current = current.parent == null ? null : byName.get(current.parent);
        }
        return false;
    }

    private static BufferedImage toBufferedImage(WritableImage image) {
        int width = (int) image.getWidth();
        int height = (int) image.getHeight();
        BufferedImage result = new BufferedImage(
                width, height, BufferedImage.TYPE_INT_ARGB);
        PixelReader reader = image.getPixelReader();
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                result.setRGB(x, y, reader.getArgb(x, y));
            }
        }
        return result;
    }
}
