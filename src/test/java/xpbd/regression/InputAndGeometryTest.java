package xpbd.regression;

import com.google.gson.JsonParser;
import constraints.DistanceConstraint;
import core.XPBDEngine;
import models.Particle;
import models.Vector3;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import xpbd.baker.BonePoseCalculator;
import xpbd.baker.CubeGeometry;
import xpbd.baker.RotationUtil;
import xpbd.export.AnimationExporter;
import xpbd.loader.BedrockAnimationData;
import xpbd.loader.BedrockModelData;

import java.util.List;
import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.*;

final class InputAndGeometryTest {
    @TempDir
    Path temporaryDirectory;

    @Test
    void scalarInfinityIsRejected() {
        assertThrows(IllegalArgumentException.class, () ->
                BedrockAnimationData.Keyframes.fromJson(
                        JsonParser.parseString("1e309")));
    }

    @Test
    void bedrockPreAndPostValuesPreserveIntervalAndKeyframeJump() {
        BedrockAnimationData.AnimationRoot root =
                BedrockAnimationData.AnimationRoot.fromJson(JsonParser.parseString("""
                        {
                          "format_version": "1.8.0",
                          "animations": {
                            "animation.jump": {
                              "animation_length": 1,
                              "bones": {
                                "arm": {
                                  "position": {
                                    "0": [0, 0, 0],
                                    "1": {"pre": [10, 0, 0], "post": [20, 0, 0]}
                                  }
                                }
                              }
                            }
                          }
                        }
                        """).getAsJsonObject());
        BedrockAnimationData.Keyframes position = root.animations
                .get("animation.jump").bones.get("arm").position;

        assertArrayEquals(new double[]{5, 0, 0}, position.evaluate(0.5), 1e-12);
        assertArrayEquals(new double[]{20, 0, 0}, position.evaluate(1.0), 1e-12);
    }

    @Test
    void malformedKeyframeReportsAnimationBoneChannelAndTime() {
        IllegalArgumentException error = assertThrows(IllegalArgumentException.class,
                () -> BedrockAnimationData.AnimationRoot.fromJson(
                        JsonParser.parseString("""
                                {
                                  "format_version": "1.8.0",
                                  "animations": {
                                    "animation.bad": {
                                      "bones": {"arm": {"rotation": {"0.25": [1, 2]}}}
                                    }
                                  }
                                }
                                """).getAsJsonObject()));

        assertTrue(error.getMessage().contains("animation.bad"));
        assertTrue(error.getMessage().contains("arm"));
        assertTrue(error.getMessage().contains("rotation"));
        assertTrue(error.getMessage().contains("0.25"));
    }

    @Test
    void unsupportedInterpolationAndWrongVectorLengthsAreRejected() {
        assertThrows(IllegalArgumentException.class, () ->
                BedrockAnimationData.Keyframes.fromJson(JsonParser.parseString(
                        "{\"0\":{\"post\":[1,2,3],\"lerp_mode\":\"bezier\"}}")));
        assertThrows(IllegalArgumentException.class, () ->
                BedrockAnimationData.Keyframes.fromJson(
                        JsonParser.parseString("[1,2,3,4]")));
    }

    @Test
    void exporterPreservesDistinctPreAndPostValues() throws Exception {
        BedrockAnimationData.AnimationRoot root =
                BedrockAnimationData.AnimationRoot.fromJson(JsonParser.parseString("""
                        {
                          "format_version": "1.8.0",
                          "animations": {
                            "animation.jump": {
                              "bones": {"arm": {"position": {
                                "1": {"pre": [10, 0, 0], "post": [20, 0, 0]}
                              }}}
                            }
                          }
                        }
                        """).getAsJsonObject());
        Path output = temporaryDirectory.resolve("pre-post.animation.json");

        AnimationExporter.export("animation.jump", root.animations.get("animation.jump"),
                List.of(), false, output.toString());

        String json = Files.readString(output);
        assertTrue(json.contains("\"pre\""));
        assertTrue(json.contains("\"post\""));
        assertTrue(json.contains("20.0"));
    }

    @Test
    void loopBehaviorIsStrictAndPreservesHoldLast() {
        BedrockAnimationData.Animation hold = BedrockAnimationData.Animation.fromJson(
                JsonParser.parseString("{\"loop\":\"hold_on_last_frame\"}")
                        .getAsJsonObject());
        assertFalse(hold.loop);
        assertEquals(BedrockAnimationData.Animation.LoopBehavior.HOLD_LAST,
                hold.loopBehavior);
        assertThrows(IllegalArgumentException.class, () ->
                BedrockAnimationData.Animation.fromJson(
                        JsonParser.parseString("{\"loop\":\"forever\"}")
                                .getAsJsonObject()));
    }

    @Test
    void exporterPreservesHoldLastLoopBehavior() throws Exception {
        BedrockAnimationData.Animation source = new BedrockAnimationData.Animation();
        source.loopBehavior = BedrockAnimationData.Animation.LoopBehavior.HOLD_LAST;
        Path output = temporaryDirectory.resolve("hold.animation.json");

        AnimationExporter.export("animation.test", source, List.of(),
                source.loopBehavior, output.toString());

        String json = Files.readString(output);
        assertTrue(json.contains("\"loop\": \"hold_on_last_frame\""));
    }

    @Test
    void exporterKeepsDistinctSubFourDecimalFrameTimes() throws Exception {
        BedrockAnimationData.Animation source = new BedrockAnimationData.Animation();
        List<xpbd.baker.PhysicsBaker.BakedFrame> frames = List.of(
                new xpbd.baker.PhysicsBaker.BakedFrame(0.00001,
                        List.of(new xpbd.baker.PhysicsBaker.BoneState(
                                "tip", new double[]{1, 0, 0}, new double[3]))),
                new xpbd.baker.PhysicsBaker.BakedFrame(0.00002,
                        List.of(new xpbd.baker.PhysicsBaker.BoneState(
                                "tip", new double[]{2, 0, 0}, new double[3]))));
        Path output = temporaryDirectory.resolve("tiny-step.animation.json");

        AnimationExporter.export("animation.tiny", source, frames,
                false, output.toString());

        var animation = JsonParser.parseString(Files.readString(output))
                .getAsJsonObject().getAsJsonObject("animations")
                .getAsJsonObject("animation.tiny");
        var positions = animation.getAsJsonObject("bones")
                .getAsJsonObject("tip").getAsJsonObject("position");
        assertEquals(2, positions.size());
        assertTrue(positions.has("0.00001"));
        assertTrue(positions.has("0.00002"));
        assertEquals(0.00002,
                animation.get("animation_length").getAsDouble(), 1e-12);
    }

    @Test
    void particleRequiresFiniteInverseMass() {
        assertThrows(IllegalArgumentException.class, () -> new Particle(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> new Particle(-1));
        assertThrows(IllegalArgumentException.class, () -> new Particle(Double.MIN_VALUE));
        assertTrue(new Particle(0).isFixed());
        assertFalse(new Particle(1).isFixed());
    }

    @Test
    void collapsedPositiveDistanceConstraintRecoversDeterministically() {
        XPBDEngine engine = new XPBDEngine();
        engine.setGravity(new Vector3());
        Particle fixed = new Particle(0);
        Particle dynamic = new Particle(1);
        engine.addParticle(fixed);
        engine.addParticle(dynamic);
        engine.addConstraint(new DistanceConstraint(0, 1, 1, 0, 0));

        engine.step(1.0 / 60.0);

        assertEquals(1, dynamic.getPosition().x, 1e-9);
        assertEquals(0, dynamic.getPosition().y, 1e-12);
        assertEquals(0, dynamic.getPosition().z, 1e-12);
    }

    @Test
    void cubeWithoutPivotRotatesAroundItsCenter() {
        BedrockModelData.Cube cube = new BedrockModelData.Cube();
        cube.origin = new double[]{0, 0, 0};
        cube.size = new double[]{2, 2, 2};
        cube.rotation = new double[]{0, 0, 90};

        assertArrayEquals(new double[]{1, 1, 1},
                CubeGeometry.effectivePivot(cube), 1e-12);
        double[] vertices = CubeGeometry.bindVertices(cube);
        assertArrayEquals(new double[]{0, 2, 0},
                new double[]{vertices[3], vertices[4], vertices[5]}, 1e-9);
    }

    @Test
    void negativeInflateShrinksBoundsWithoutMovingCenter() {
        BedrockModelData.Cube cube = new BedrockModelData.Cube();
        cube.origin = new double[]{1, 2, 3};
        cube.size = new double[]{4, 6, 8};
        cube.inflate = -0.25;

        assertArrayEquals(new double[]{1.25, 2.25, 3.25},
                CubeGeometry.effectiveOrigin(cube), 1e-12);
        assertArrayEquals(new double[]{3.5, 5.5, 7.5},
                CubeGeometry.effectiveSize(cube), 1e-12);
        assertArrayEquals(new double[]{3, 5, 7},
                CubeGeometry.effectivePivot(cube), 1e-12);
        double[] vertices = CubeGeometry.bindVertices(cube);
        assertArrayEquals(new double[]{-4.75, 2.25, 3.25},
                new double[]{vertices[0], vertices[1], vertices[2]}, 1e-12);
        assertArrayEquals(new double[]{-1.25, 7.75, 10.75},
                new double[]{vertices[21], vertices[22], vertices[23]}, 1e-12);
    }

    @Test
    void inflateCannotShrinkAnAxisBelowZero() {
        assertThrows(IllegalArgumentException.class, () ->
                BedrockModelData.Cube.fromJson(JsonParser.parseString("""
                        {"origin":[0,0,0],"size":[1,1,1],"inflate":-0.6}
                        """).getAsJsonObject()));
    }

    @Test
    void bedrockPositiveEulerRotatesXAxisTowardPositiveY() {
        BedrockModelData.Bone bone = new BedrockModelData.Bone();
        bone.name = "root";
        bone.rotation = new double[]{0, 0, 90};
        BonePoseCalculator.Pose pose = BonePoseCalculator
                .calculate(List.of(bone), null, 0).get("root");
        double[] result = new double[3];

        CubeGeometry.transformPoint(pose, 1, 0, 0, result);

        assertArrayEquals(new double[]{0, 1, 0}, result, 1e-9);
    }

    @Test
    void bedrockEulerRoundTripsRepresentativeModelRotation() {
        double[] modelRotation = new double[]{2.5, 0, -12.5};
        double[] internal = RotationUtil.quaternionFromBedrockEuler(
                modelRotation[0], modelRotation[1], modelRotation[2]);

        assertArrayEquals(modelRotation,
                RotationUtil.bedrockEulerFromQuaternion(internal), 1e-9);
    }

    @Test
    void eulerUnwrapChoosesNearestEquivalentAcrossHalfTurn() {
        double[] first = new double[]{0, 0, 179};
        double[] second = RotationUtil.unwrapEuler(first, new double[]{0, 0, -179});
        double[] third = RotationUtil.unwrapEuler(second, new double[]{0, 0, -150});

        assertArrayEquals(new double[]{0, 0, 181}, second, 1e-12);
        assertArrayEquals(new double[]{0, 0, 210}, third, 1e-12);
    }
}
