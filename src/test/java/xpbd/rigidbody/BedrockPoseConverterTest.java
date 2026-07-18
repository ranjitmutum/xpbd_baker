package xpbd.rigidbody;

import org.junit.jupiter.api.Test;
import xpbd.baker.BonePoseCalculator;
import xpbd.loader.BedrockModelData;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

final class BedrockPoseConverterTest {
    @Test
    void hierarchicalPivotPoseRoundTripsToLocalChannels() {
        BedrockModelData.Bone parent = new BedrockModelData.Bone();
        parent.name = "body";
        parent.pivot = new double[]{1, 10, -2};
        parent.rotation = new double[]{5, -8, 12};
        BedrockModelData.Bone child = new BedrockModelData.Bone();
        child.name = "ribbon";
        child.parent = parent.name;
        child.pivot = new double[]{2, 6, -1};
        child.rotation = new double[]{-3, 4, 7};
        double[] parentPosition = new double[]{0.4, -0.2, 0.8};
        double[] parentRotation = new double[]{3, -2, 5};
        double[] childPosition = new double[]{-0.3, 0.7, 0.2};
        double[] childRotation = new double[]{6, -5, 9};
        List<BedrockModelData.Bone> bones = List.of(parent, child);
        Map<String, BonePoseCalculator.Pose> poses = BonePoseCalculator.calculate(
                bones, null, 0,
                Map.of(parent.name, parentPosition, child.name, childPosition),
                Map.of(parent.name, parentRotation, child.name, childRotation));
        double unitScale = 1.0 / 16.0;

        BedrockPoseConverter.LocalChannels converted =
                BedrockPoseConverter.toLocalChannels(child,
                        BedrockPoseConverter.fromPose(poses.get(child.name), unitScale),
                        parent,
                        BedrockPoseConverter.fromPose(poses.get(parent.name), unitScale),
                        unitScale);

        assertArrayEquals(childPosition, converted.position(), 2e-6);
        assertArrayEquals(childRotation, converted.rotation(), 2e-6);

        Map<String, BonePoseCalculator.Pose> rebuilt = BonePoseCalculator.calculate(
                bones, null, 0,
                Map.of(parent.name, parentPosition,
                        child.name, converted.position()),
                Map.of(parent.name, parentRotation,
                        child.name, converted.rotation()));
        BonePoseCalculator.Pose expected = poses.get(child.name);
        BonePoseCalculator.Pose actual = rebuilt.get(child.name);
        assertArrayEquals(expected.worldPosition, actual.worldPosition, 2e-6);
        assertQuaternionEquivalent(expected.worldRotation, actual.worldRotation, 2e-6);
    }

    private static void assertQuaternionEquivalent(
            double[] expected, double[] actual, double tolerance) {
        double dot = 0;
        for (int i = 0; i < 4; i++) dot += expected[i] * actual[i];
        double sign = dot < 0 ? -1 : 1;
        for (int i = 0; i < 4; i++) {
            assertEquals(expected[i], actual[i] * sign, tolerance);
        }
    }
}
