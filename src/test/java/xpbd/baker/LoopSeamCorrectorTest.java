package xpbd.baker;

import org.junit.jupiter.api.Test;
import xpbd.loader.BedrockModelData;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

final class LoopSeamCorrectorTest {
    @Test
    void quinticCorrectionClosesValueVelocityAndAcceleration() {
        BedrockModelData.Bone bone = bone("tip");
        List<PhysicsBaker.BakedFrame> source = scalarFrames();

        LoopSeamCorrector.Result result = LoopSeamCorrector.correctCopy(source,
                Map.of(bone.name, bone), Set.of(bone.name), 0.5, true);
        List<PhysicsBaker.BakedFrame> frames = result.frames();
        int last = frames.size() - 1;

        assertArrayEquals(position(frames, 0), position(frames, last), 0);
        assertTrue(Math.abs(velocity(frames, 0, 1)
                - velocity(frames, last - 1, last)) < 1e-9);
        assertTrue(Math.abs(acceleration(frames, 0, 1, 2)
                - acceleration(frames, last - 2, last - 1, last)) < 1e-8);
        assertArrayEquals(position(source, result.windowStartIndex()),
                position(frames, result.windowStartIndex()), 0,
                "the correction must enter with zero displacement");
    }

    @Test
    void quaternionCorrectionUsesTheShortArcAcrossTheEulerBranch() {
        BedrockModelData.Bone bone = bone("spinner");
        List<PhysicsBaker.BakedFrame> frames = new ArrayList<>();
        for (int i = 0; i <= 10; i++) {
            double angle = 170 + 20.0 * i / 10.0;
            frames.add(frame(i / 10.0, bone.name,
                    new double[]{0, 0, 0}, new double[]{0, 0, angle}));
        }

        List<PhysicsBaker.BakedFrame> corrected = LoopSeamCorrector.correctCopy(
                frames, Map.of(bone.name, bone), Set.of(bone.name), 0.5, true).frames();
        int last = corrected.size() - 1;
        assertArrayEquals(corrected.get(0).getBoneState(bone.name).rotation,
                corrected.get(last).getBoneState(bone.name).rotation, 0);
        for (int i = 1; i < corrected.size(); i++) {
            double[] a = totalQuaternion(bone, corrected.get(i - 1));
            double[] b = totalQuaternion(bone, corrected.get(i));
            double step = length(RotationUtil.rotationVectorFromQuaternion(
                    RotationUtil.quaternionMultiply(
                            RotationUtil.quaternionInverse(a), b)));
            assertTrue(step < Math.toRadians(30), "rotation took a long arc at " + i);
        }
    }

    private static List<PhysicsBaker.BakedFrame> scalarFrames() {
        List<PhysicsBaker.BakedFrame> frames = new ArrayList<>();
        for (int i = 0; i <= 10; i++) {
            double t = i / 10.0;
            double x = t * (1 - t);
            frames.add(frame(t, "tip", new double[]{x, 0, 0},
                    new double[]{0, 0, 0}));
        }
        return frames;
    }

    private static PhysicsBaker.BakedFrame frame(double time, String name,
                                                  double[] position, double[] rotation) {
        return new PhysicsBaker.BakedFrame(time, List.of(
                new PhysicsBaker.BoneState(name, position, rotation,
                        new double[3], new double[3])));
    }

    private static BedrockModelData.Bone bone(String name) {
        BedrockModelData.Bone bone = new BedrockModelData.Bone();
        bone.name = name;
        return bone;
    }

    private static double[] position(List<PhysicsBaker.BakedFrame> frames, int i) {
        return frames.get(i).getBoneState("tip").position;
    }

    private static double velocity(List<PhysicsBaker.BakedFrame> frames, int a, int b) {
        return (position(frames, b)[0] - position(frames, a)[0])
                / (frames.get(b).time - frames.get(a).time);
    }

    private static double acceleration(List<PhysicsBaker.BakedFrame> frames,
                                       int a, int b, int c) {
        double first = velocity(frames, a, b);
        double second = velocity(frames, b, c);
        return (second - first) / (0.5 * (frames.get(c).time - frames.get(a).time));
    }

    private static double[] totalQuaternion(BedrockModelData.Bone bone,
                                            PhysicsBaker.BakedFrame frame) {
        double[] r = frame.getBoneState(bone.name).rotation;
        return RotationUtil.quaternionFromBedrockEuler(
                bone.rotation[0] + r[0], bone.rotation[1] + r[1],
                bone.rotation[2] + r[2]);
    }

    private static double length(double[] value) {
        return Math.sqrt(value[0] * value[0] + value[1] * value[1]
                + value[2] * value[2]);
    }
}
