package xpbd.loader;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import xpbd.baker.PhysicsBaker;
import xpbd.export.AnimationExporter;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.*;

final class MolangKeyframeDetectorTest {
    @TempDir
    Path temporaryDirectory;

    @Test
    void checksOnlyThePassedAnimationAndSelectedBones() {
        BedrockAnimationData.AnimationRoot root = parseRoot("""
                {
                  "format_version": "1.8.0",
                  "animations": {
                    "animation.current": {
                      "bones": {
                        "selected": {"position": [1, 2, 3]},
                        "not_selected": {"rotation": ["query.anim_time", 0, 0]}
                      }
                    },
                    "animation.other": {
                      "bones": {
                        "selected": {"scale": "query.life_time"}
                      }
                    }
                  }
                }
                """);
        BedrockAnimationData.Animation current =
                root.animations.get("animation.current");
        BedrockAnimationData.Animation other =
                root.animations.get("animation.other");

        assertFalse(MolangKeyframeDetector.hasMolangKeyframes(
                current, Set.of("selected")));
        assertTrue(MolangKeyframeDetector.hasMolangKeyframes(
                current, Set.of("not_selected")));
        assertTrue(MolangKeyframeDetector.hasMolangKeyframes(
                other, Set.of("selected")));
        assertFalse(MolangKeyframeDetector.hasMolangKeyframes(
                null, Set.of("selected")));
        assertFalse(MolangKeyframeDetector.hasMolangKeyframes(
                current, Set.of()));
    }

    @Test
    void detectsScalarVectorAndCompositeMolangChannels() {
        BedrockAnimationData.Animation animation = parseRoot("""
                {
                  "format_version": "1.8.0",
                  "animations": {
                    "animation.test": {
                      "bones": {
                        "physics": {
                          "position": "query.anim_time * 10",
                          "rotation": ["query.anim_time", 0, 0],
                          "scale": {
                            "0": {
                              "pre": [1, 1, 1],
                              "post": ["variable.scale", 1, 1],
                              "lerp_mode": "linear"
                            }
                          }
                        }
                      }
                    }
                  }
                }
                """).animations.get("animation.test");
        BedrockAnimationData.BoneAnimation physics =
                animation.bones.get("physics");

        assertTrue(physics.position.containsMolang());
        assertTrue(physics.rotation.containsMolang());
        assertTrue(physics.scale.containsMolang());
        assertTrue(MolangKeyframeDetector.hasMolangKeyframes(
                animation, Set.of("physics")));
        assertArrayEquals(new double[]{0, 0, 0},
                physics.position.evaluate(0), 1e-12);
        assertArrayEquals(new double[]{0, 0, 0},
                physics.rotation.evaluate(0), 1e-12);
    }

    @Test
    void numericStringsAndInterpolationMetadataAreNotMolang() {
        BedrockAnimationData.Keyframes numeric =
                BedrockAnimationData.Keyframes.fromJson(JsonParser.parseString("""
                        {"0": {"post": ["1", "2", "3"],
                                "lerp_mode": "catmullrom"}}
                        """));

        assertFalse(numeric.containsMolang());
        assertFalse(BedrockAnimationData.Keyframes.isMolangValue(
                JsonParser.parseString("{\"time\":0,\"lerp_mode\":\"linear\","
                        + "\"data\":[1,2,3]}")));
        assertTrue(BedrockAnimationData.Keyframes.isMolangValue(
                JsonParser.parseString("{\"post\":[1,\"query.anim_time\",3]}")));
    }

    @Test
    void exporterPreservesUnbakedMolangAndOverwritesBakedChannels() throws Exception {
        BedrockAnimationData.Animation source = parseRoot("""
                {
                  "format_version": "1.8.0",
                  "animations": {
                    "animation.test": {
                      "animation_length": 1,
                      "bones": {
                        "physics": {
                          "position": ["query.anim_time", 0, 0],
                          "rotation": [0, "variable.turn", 0]
                        },
                        "not_selected": {
                          "rotation": [0, 0, "query.anim_time * 10"]
                        }
                      }
                    }
                  }
                }
                """).animations.get("animation.test");
        PhysicsBaker.BoneState bakedState = new PhysicsBaker.BoneState(
                "physics", new double[]{4, 5, 6}, new double[]{7, 8, 9});
        List<PhysicsBaker.BakedFrame> frames = List.of(
                new PhysicsBaker.BakedFrame(0, List.of(bakedState)));
        Path output = temporaryDirectory.resolve("molang.animation.json");

        AnimationExporter.export("animation.test", source, frames, false,
                output.toString());

        JsonObject exported = JsonParser.parseString(Files.readString(output))
                .getAsJsonObject().getAsJsonObject("animations")
                .getAsJsonObject("animation.test").getAsJsonObject("bones");
        assertEquals(4, exported.getAsJsonObject("physics")
                .getAsJsonObject("position").getAsJsonArray("0.0000").get(0)
                .getAsDouble());
        assertEquals("query.anim_time * 10", exported
                .getAsJsonObject("not_selected").getAsJsonArray("rotation")
                .get(2).getAsString());
    }

    private static BedrockAnimationData.AnimationRoot parseRoot(String json) {
        return BedrockAnimationData.AnimationRoot.fromJson(
                JsonParser.parseString(json).getAsJsonObject());
    }
}
