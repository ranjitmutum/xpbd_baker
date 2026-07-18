package xpbd.baker;

import org.junit.jupiter.api.Test;
import xpbd.loader.BedrockAnimationData;
import xpbd.loader.BedrockModelData;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

final class BonePoseCalculatorTest {
    @Test
    void compiledEvaluatorMatchesReferenceWithUnorderedHierarchyAndOverrides() {
        BedrockModelData.Bone parent = bone("parent", null,
                new double[]{1, 2, 3}, new double[]{5, 10, 15});
        BedrockModelData.Bone child = bone("child", parent.name,
                new double[]{4, 5, 6}, new double[]{-10, 20, 30});
        List<BedrockModelData.Bone> unordered = List.of(child, parent);

        BedrockAnimationData.Animation animation = new BedrockAnimationData.Animation();
        BedrockAnimationData.BoneAnimation channel =
                new BedrockAnimationData.BoneAnimation();
        channel.position = new BedrockAnimationData.Keyframes();
        channel.position.keyframes.put(0.0, new double[]{0, 0, 0});
        channel.position.keyframes.put(1.0, new double[]{2, 4, 6});
        channel.rotation = new BedrockAnimationData.Keyframes();
        channel.rotation.keyframes.put(0.0, new double[]{0, 0, 0});
        channel.rotation.keyframes.put(1.0, new double[]{20, 40, 60});
        animation.bones.put(child.name, channel);
        Map<String, double[]> positionOverrides = Map.of(
                parent.name, new double[]{3, 2, 1});
        Map<String, double[]> rotationOverrides = Map.of(
                child.name, new double[]{9, 8, 7});

        Map<String, BonePoseCalculator.Pose> reference =
                BonePoseCalculator.calculate(unordered, animation, 0.5,
                        positionOverrides, rotationOverrides);
        Map<String, BonePoseCalculator.Pose> compiled =
                BonePoseCalculator.compile(unordered).calculate(animation, 0.5,
                        positionOverrides, rotationOverrides);

        assertEquals(reference.keySet(), compiled.keySet());
        for (String name : reference.keySet()) {
            assertPoseEquals(reference.get(name), compiled.get(name));
        }
    }

    @Test
    void compiledEvaluatorRejectsHierarchyCycle() {
        BedrockModelData.Bone first = bone("first", "second",
                new double[3], new double[3]);
        BedrockModelData.Bone second = bone("second", "first",
                new double[3], new double[3]);

        assertThrows(IllegalArgumentException.class,
                () -> BonePoseCalculator.compile(List.of(first, second)));
    }

    private static BedrockModelData.Bone bone(String name, String parent,
                                               double[] pivot, double[] rotation) {
        BedrockModelData.Bone bone = new BedrockModelData.Bone();
        bone.name = name;
        bone.parent = parent;
        bone.pivot = pivot;
        bone.rotation = rotation;
        return bone;
    }

    private static void assertPoseEquals(BonePoseCalculator.Pose expected,
                                         BonePoseCalculator.Pose actual) {
        assertArrayEquals(expected.worldPosition, actual.worldPosition, 1e-12);
        assertArrayEquals(expected.worldRotation, actual.worldRotation, 1e-12);
        assertArrayEquals(expected.worldTranslation, actual.worldTranslation, 1e-12);
        assertArrayEquals(expected.animationPosition, actual.animationPosition, 1e-12);
        assertArrayEquals(expected.totalLocalEuler, actual.totalLocalEuler, 1e-12);
    }
}
