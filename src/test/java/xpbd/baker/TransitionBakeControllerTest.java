package xpbd.baker;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import xpbd.export.AnimationExporter;
import xpbd.loader.BedrockAnimationData;
import xpbd.loader.BedrockModelData;

import java.util.List;
import java.util.Map;
import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

final class TransitionBakeControllerTest {
    @Test
    void startsAtAAndCriticallyDampsExactlyToB() {
        BedrockModelData.Bone bone = new BedrockModelData.Bone();
        bone.name = "root";
        BedrockAnimationData.Animation source = animation(
                bone.name, new double[]{10, 0, 0}, new double[]{0, 170, 0});
        BedrockAnimationData.Animation target = animation(
                bone.name, new double[]{0, 0, 0}, new double[]{0, -170, 0});
        TransitionBakeController controller = new TransitionBakeController(
                new TransitionBakeRequest(source, target,
                        1, 0, 1, Map.of()), List.of(bone));

        BonePoseCalculator.Pose start = controller.sample(0).get(bone.name);
        BonePoseCalculator.Pose middle = controller.sample(0.5).get(bone.name);
        BonePoseCalculator.Pose end = controller.sample(1).get(bone.name);

        assertEquals(10, start.animationPosition[0], 1e-9);
        assertTrue(middle.animationPosition[0] > end.animationPosition[0]);
        assertEquals(0, end.animationPosition[0], 1e-12,
                "the final baked keyframe must return exactly to B");
        assertEquals(0, PeriodicStateAdapter.compareSnapshots(
                snapshot(start.worldRotation), snapshot(
                        RotationUtil.quaternionFromBedrockEuler(0, 170, 0)))
                .maximumRotationErrorRadians(), 1e-9);
        double targetAngle = quaternionAngle(end.worldRotation,
                RotationUtil.quaternionFromBedrockEuler(0, -170, 0));
        assertEquals(0, targetAngle, 1e-9,
                "the final baked rotation must return exactly to B");
    }

    @Test
    void targetEntryTimeChangesTheSampledTargetTimeline() {
        BedrockModelData.Bone bone = new BedrockModelData.Bone();
        bone.name = "root";
        BedrockAnimationData.Animation source = animation(
                bone.name, new double[]{0, 0, 0}, new double[3]);
        BedrockAnimationData.Animation target = movingAnimation(
                bone.name, new double[]{0, 0, 0}, new double[]{10, 0, 0},
                new double[3], new double[3]);
        TransitionBakeController fromStart = new TransitionBakeController(
                new TransitionBakeRequest(source, target,
                        0.5, 0, 0.25, Map.of()), List.of(bone));
        TransitionBakeController fromMiddle = new TransitionBakeController(
                new TransitionBakeRequest(source, target,
                        0.5, 0.5, 0.25, Map.of()), List.of(bone));

        assertEquals(2.5, fromStart.sample(0.25).get(bone.name)
                .animationPosition[0], 1e-9);
        assertEquals(7.5, fromMiddle.sample(0.25).get(bone.name)
                .animationPosition[0], 1e-9);
    }

    @Test
    void perBoneFollowWeightsZeroHalfAndOneHaveOrderedResponses() {
        BedrockModelData.Bone bone = new BedrockModelData.Bone();
        bone.name = "root";
        BedrockAnimationData.Animation source = animation(
                bone.name, new double[]{10, 0, 0}, new double[3]);
        BedrockAnimationData.Animation target = animation(
                bone.name, new double[]{0, 0, 0}, new double[3]);

        double noFollow = samplePosition(source, target, bone, 0.0);
        double halfFollow = samplePosition(source, target, bone, 0.5);
        double fullFollow = samplePosition(source, target, bone, 1.0);

        assertEquals(10, noFollow, 1e-9);
        assertTrue(noFollow > halfFollow);
        assertTrue(halfFollow > fullFollow);
    }

    @Test
    void explicitXpbdRequestUsesNonBoundaryExitAndModeSwitchClearsIt() {
        BedrockModelData.Bone bone = new BedrockModelData.Bone();
        bone.name = "root";
        BoneMapper mapper = new BoneMapper(List.of(bone));
        mapper.addPhysicsBone(bone.name);
        mapper.getConfig().gravityY = 0;
        mapper.getConfig().windSpeed = 0;
        mapper.getConfig().airDrag = 0;
        mapper.getConfig().turbulence = 0;
        mapper.getConfig().transitionDuration = 0.05;
        BedrockAnimationData.Animation source = blankAnimation(0.1);
        BedrockAnimationData.Animation target = blankAnimation(0.1);

        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setDt(0.01);
            baker.setSourceAnimation(source);
            baker.setTransitionAnimation(null);
            baker.setTransitionRequest(new TransitionBakeRequest(
                    source, target, 0.02, 0.03, 0.04, Map.of()));
            baker.initialize();
            assertEquals(6, baker.getTotalSteps(),
                    "custom pre-roll must use the requested non-boundary exit time");
            baker.runToEnd();
            assertEquals(0.04, baker.getFrames().get(
                    baker.getFrames().size() - 1).time, 1e-12);

            baker.reset();
            baker.setTransitionRequest(null);
            baker.setTransitionAnimation(target);
            baker.initialize();
            assertEquals(15, baker.getTotalSteps(),
                    "simple mode must return to source-end plus configured duration");
        }
    }

    @Test
    void transitionQuaternionReferenceIncludesBoneBaseRotation() {
        BedrockModelData.Bone bone = new BedrockModelData.Bone();
        bone.name = "root";
        bone.rotation = new double[]{25, -15, 35};
        double[] sourceChannel = new double[]{10, 20, -30};
        double[] targetChannel = new double[]{-40, 15, 50};
        BedrockAnimationData.Animation source = animation(
                bone.name, new double[3], sourceChannel);
        BedrockAnimationData.Animation target = animation(
                bone.name, new double[3], targetChannel);
        TransitionBakeController controller = new TransitionBakeController(
                new TransitionBakeRequest(source, target,
                        1, 0, 1, Map.of()), List.of(bone));

        double[] sourceQ = RotationUtil.quaternionFromBedrockEuler(
                bone.rotation[0] + sourceChannel[0],
                bone.rotation[1] + sourceChannel[1],
                bone.rotation[2] + sourceChannel[2]);
        double[] targetQ = RotationUtil.quaternionFromBedrockEuler(
                bone.rotation[0] + targetChannel[0],
                bone.rotation[1] + targetChannel[1],
                bone.rotation[2] + targetChannel[2]);
        double[] offset = RotationUtil.rotationVectorFromQuaternion(
                RotationUtil.quaternionMultiply(sourceQ,
                        RotationUtil.quaternionInverse(targetQ)));
        double[] expectedOffset = new InertializedTarget(
                offset, new double[3], 1, 1).offsetAt(0.5);
        double[] expected = RotationUtil.quaternionMultiply(
                RotationUtil.quaternionFromRotationVector(expectedOffset), targetQ);
        double[] actual = controller.sample(0.5).get(bone.name).worldRotation;

        assertEquals(0, quaternionAngle(expected, actual), 1e-9,
                "transition math must operate in total local rotation space");
    }

    @Test
    void physicalSwitchUsesLiveVelocityInsteadOfLaggingSourceVelocity() {
        BedrockModelData.Bone bone = new BedrockModelData.Bone();
        bone.name = "root";
        BedrockAnimationData.Animation source = animation(
                bone.name, new double[]{0, 0, 0}, new double[]{0, 0, 0});
        BedrockAnimationData.Animation target = animation(
                bone.name, new double[]{0, 0, 0}, new double[]{0, 0, 0});
        PhysicsBaker.BakedFrame physical = new PhysicsBaker.BakedFrame(0,
                List.of(new PhysicsBaker.BoneState(bone.name,
                        new double[]{10, 0, 0}, new double[]{0, 0, 90},
                        new double[]{10, 0, 0}, new double[]{10, 0, 0})));
        PhysicsBaker.BakedFrame previousPhysical = new PhysicsBaker.BakedFrame(-0.1,
                List.of(new PhysicsBaker.BoneState(bone.name,
                        new double[]{9, 0, 0}, new double[]{0, 0, 81},
                        new double[]{10, 0, 0}, new double[]{9, 0, 0})));
        TransitionBakeController controller = new TransitionBakeController(
                new TransitionBakeRequest(source, target,
                        1, 0, 1, Map.of()), List.of(bone), physical,
                previousPhysical, 0.1);
        double h = 1e-6;
        BonePoseCalculator.Pose start = controller.sample(0).get(bone.name);
        BonePoseCalculator.Pose next = controller.sample(h).get(bone.name);

        assertEquals(10, (next.animationPosition[0]
                - start.animationPosition[0]) / h, 2e-3);
        assertEquals(Math.PI / 2,
                quaternionAngle(start.worldRotation, next.worldRotation) / h,
                2e-3);
    }

    @Test
    void livePhysicalPoseOverridesLaggingSourceTargetAtSwitch() {
        BedrockModelData.Bone bone = new BedrockModelData.Bone();
        bone.name = "root";
        BedrockAnimationData.Animation source = animation(
                bone.name, new double[]{10, 0, 0}, new double[]{0, 90, 0});
        BedrockAnimationData.Animation target = animation(
                bone.name, new double[]{0, 0, 0}, new double[]{0, 0, 0});
        PhysicsBaker.BakedFrame physical = new PhysicsBaker.BakedFrame(0,
                List.of(new PhysicsBaker.BoneState(bone.name,
                        new double[]{4, 0, 0}, new double[]{0, 45, 0},
                        new double[]{2, 0, 0}, new double[]{4, 0, 0})));
        TransitionBakeController controller = new TransitionBakeController(
                new TransitionBakeRequest(source, target,
                        1, 0, 1, Map.of()), List.of(bone), physical);

        BonePoseCalculator.Pose start = controller.sample(0).get(bone.name);

        assertEquals(4, start.animationPosition[0], 1e-9);
        assertTrue(quaternionAngle(start.worldRotation,
                RotationUtil.quaternionFromBedrockEuler(0, 45, 0)) < 1e-9);
    }

    @Test
    void physicsBakerKeepsXpbdVelocityAcrossTheSwitch() {
        BedrockModelData.Bone falling = new BedrockModelData.Bone();
        falling.name = "falling";
        BoneMapper mapper = new BoneMapper(List.of(falling));
        mapper.addPhysicsBone(falling.name);
        BoneMapper.BonePhysicsConfig config = new BoneMapper.BonePhysicsConfig();
        config.fixed = false;
        mapper.setBoneConfig(falling.name, config);
        mapper.getConfig().gravityY = -10;
        mapper.getConfig().airDrag = 0;
        mapper.getConfig().turbulence = 0;
        mapper.getConfig().animationPullCompliance = 0;
        mapper.getConfig().transitionDuration = 0.1;
        BedrockAnimationData.Animation source = blankAnimation(0.1);
        BedrockAnimationData.Animation target = blankAnimation(0.1);

        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setSourceAnimation(source);
            baker.setTransitionAnimation(target);
            baker.initialize();
            baker.runToEnd();

            PhysicsBaker.BoneState first = baker.getFrames().get(0)
                    .getBoneState(falling.name);
            PhysicsBaker.BoneState next = baker.getFrames().get(1)
                    .getBoneState(falling.name);
            assertTrue(first.linearVelocity[1] < -0.5,
                    "A's falling velocity must survive the switch frame");
            assertTrue(next.linearVelocity[1] < first.linearVelocity[1],
                    "B must continue from the live state instead of restarting");
        }
    }

    @Test
    void physicsBakerFinalKeyframeMatchesTargetDespiteResidualMotion() {
        BedrockModelData.Bone anchor = new BedrockModelData.Bone();
        anchor.name = "anchor";
        BedrockModelData.Bone tip = new BedrockModelData.Bone();
        tip.name = "tip";
        tip.parent = anchor.name;
        tip.pivot = new double[]{1, 0, 0};
        BoneMapper mapper = new BoneMapper(List.of(anchor, tip));
        mapper.addPhysicsBone(anchor.name);
        mapper.addPhysicsBone(tip.name);
        mapper.getConfig().gravityY = -20;
        mapper.getConfig().airDrag = 0;
        mapper.getConfig().turbulence = 0;
        mapper.getConfig().animationPullCompliance = 0;
        mapper.getConfig().transitionDuration = 0.1;
        BedrockAnimationData.Animation source = blankAnimation(0.1);
        BedrockAnimationData.Animation target = blankAnimation(0.1);
        BedrockAnimationData.BoneAnimation targetAnchor =
                new BedrockAnimationData.BoneAnimation();
        targetAnchor.rotation = new BedrockAnimationData.Keyframes();
        targetAnchor.rotation.keyframes.put(0.0, new double[]{0, 0, 30});
        target.bones.put(anchor.name, targetAnchor);

        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setDt(0.01);
            baker.setSourceAnimation(source);
            baker.setTransitionAnimation(target);
            baker.initialize();
            baker.runToEnd();

            PhysicsBaker.BoneState first = baker.getFrames().get(0)
                    .getBoneState(anchor.name);
            PhysicsBaker.BoneState last = baker.getFrames().get(
                    baker.getFrames().size() - 1).getBoneState(anchor.name);
            assertTrue(Math.abs(first.rotation[2]) > 0.1,
                    "the transition must start from A's live drooping pose");
            assertArrayEquals(new double[]{0, 0, 30}, last.rotation, 1e-9,
                    "the transition clip must end exactly on B");
        }
    }

    @Test
    void ordinaryNonLoopBakeReturnsExactlyToSameSourceAtBothEdges() {
        BedrockModelData.Bone anchor = new BedrockModelData.Bone();
        anchor.name = "anchor";
        BedrockModelData.Bone tip = new BedrockModelData.Bone();
        tip.name = "tip";
        tip.parent = anchor.name;
        tip.pivot = new double[]{0, -1, 0};
        tip.rotation = new double[]{15, -20, 25};
        BoneMapper mapper = new BoneMapper(List.of(anchor, tip));
        mapper.addPhysicsBone(anchor.name);
        mapper.addPhysicsBone(tip.name);
        mapper.getConfig().gravityY = -20;
        mapper.getConfig().windSpeed = 0;
        mapper.getConfig().airDrag = 0;
        mapper.getConfig().turbulence = 0;
        mapper.getConfig().animationPullCompliance = 0;
        mapper.getConfig().transitionDuration = 0.04;

        BedrockAnimationData.Animation source = blankAnimation(0.1);
        BedrockAnimationData.BoneAnimation tipAnimation =
                new BedrockAnimationData.BoneAnimation();
        tipAnimation.position = new BedrockAnimationData.Keyframes();
        tipAnimation.position.keyframes.put(0.0, new double[]{1, 2, 3});
        tipAnimation.position.keyframes.put(0.1, new double[]{4, 5, 6});
        tipAnimation.rotation = new BedrockAnimationData.Keyframes();
        tipAnimation.rotation.keyframes.put(0.0, new double[]{10, 20, 30});
        tipAnimation.rotation.keyframes.put(0.1, new double[]{-35, 40, -45});
        source.bones.put(tip.name, tipAnimation);

        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setDt(0.01);
            baker.setSourceAnimation(source);
            baker.setTransitionAnimation(source);
            baker.initialize();
            baker.runToEnd();

            PhysicsBaker.BoneState first = baker.getFrames().get(0)
                    .getBoneState(tip.name);
            PhysicsBaker.BoneState last = baker.getFrames().get(
                    baker.getFrames().size() - 1).getBoneState(tip.name);
            assertArrayEquals(new double[]{1, 2, 3}, first.position, 1e-9);
            assertArrayEquals(new double[]{10, 20, 30}, first.rotation, 1e-9);
            assertArrayEquals(new double[]{4, 5, 6}, last.position, 1e-9);
            assertArrayEquals(new double[]{-35, 40, -45}, last.rotation, 1e-9);
            assertArrayEquals(new double[3], first.linearVelocity, 1e-9);
            assertArrayEquals(new double[3], last.linearVelocity, 1e-9);
        }
    }

    @Test
    void transitionExportUsesBTimelineAndExactSegmentLength(@TempDir Path directory)
            throws Exception {
        BedrockAnimationData.Animation target = blankAnimation(1);
        BedrockAnimationData.BoneAnimation arm = new BedrockAnimationData.BoneAnimation();
        arm.position = new BedrockAnimationData.Keyframes();
        arm.position.keyframes.put(0.0, new double[]{0, 0, 0});
        arm.position.keyframes.put(1.0, new double[]{10, 0, 0});
        target.bones.put("arm", arm);
        List<PhysicsBaker.BakedFrame> frames = List.of(
                new PhysicsBaker.BakedFrame(0, List.of()),
                new PhysicsBaker.BakedFrame(0.25, List.of()));
        Path output = directory.resolve("transition.animation.json");

        AnimationExporter.export("animation.a_to_b", target, frames,
                BedrockAnimationData.Animation.LoopBehavior.ONCE,
                output.toString(), time -> time + 0.5, true);

        JsonObject animation = JsonParser.parseString(Files.readString(output))
                .getAsJsonObject().getAsJsonObject("animations")
                .getAsJsonObject("animation.a_to_b");
        assertEquals(0.25, animation.get("animation_length").getAsDouble(), 1e-12);
        JsonObject positions = animation.getAsJsonObject("bones")
                .getAsJsonObject("arm").getAsJsonObject("position");
        assertEquals(5, positions.getAsJsonArray("0.0000").get(0).getAsDouble(), 1e-9);
        assertEquals(7.5, positions.getAsJsonArray("0.2500").get(0).getAsDouble(), 1e-9);
    }

    private static BedrockAnimationData.Animation animation(
            String boneName, double[] position, double[] rotation) {
        BedrockAnimationData.Animation animation = blankAnimation(1);
        BedrockAnimationData.BoneAnimation bone = new BedrockAnimationData.BoneAnimation();
        bone.position = new BedrockAnimationData.Keyframes();
        bone.position.keyframes.put(0.0, position);
        bone.position.keyframes.put(1.0, position);
        bone.rotation = new BedrockAnimationData.Keyframes();
        bone.rotation.keyframes.put(0.0, rotation);
        bone.rotation.keyframes.put(1.0, rotation);
        animation.bones.put(boneName, bone);
        return animation;
    }

    private static BedrockAnimationData.Animation movingAnimation(
            String boneName, double[] startPosition, double[] endPosition,
            double[] startRotation, double[] endRotation) {
        BedrockAnimationData.Animation animation = blankAnimation(1);
        BedrockAnimationData.BoneAnimation bone = new BedrockAnimationData.BoneAnimation();
        bone.position = new BedrockAnimationData.Keyframes();
        bone.position.keyframes.put(0.0, startPosition);
        bone.position.keyframes.put(1.0, endPosition);
        bone.rotation = new BedrockAnimationData.Keyframes();
        bone.rotation.keyframes.put(0.0, startRotation);
        bone.rotation.keyframes.put(1.0, endRotation);
        animation.bones.put(boneName, bone);
        return animation;
    }

    private static double samplePosition(
            BedrockAnimationData.Animation source,
            BedrockAnimationData.Animation target,
            BedrockModelData.Bone bone, double weight) {
        TransitionBakeController controller = new TransitionBakeController(
                new TransitionBakeRequest(source, target,
                        1, 0, 1, Map.of(bone.name, weight)), List.of(bone));
        return controller.sample(0.5).get(bone.name).animationPosition[0];
    }

    private static BedrockAnimationData.Animation blankAnimation(double length) {
        BedrockAnimationData.Animation animation = new BedrockAnimationData.Animation();
        animation.animationLength = length;
        return animation;
    }

    private static PeriodicStateAdapter.Snapshot snapshot(double[] rotation) {
        return new PeriodicStateAdapter.Snapshot(Map.of("root",
                new PeriodicStateAdapter.BoneState(new double[3], rotation,
                        new double[3], null)), java.util.Set.of(), 0, 0);
    }

    private static double quaternionAngle(double[] left, double[] right) {
        double dot = 0;
        for (int i = 0; i < 4; i++) dot += left[i] * right[i];
        return 2 * Math.acos(Math.max(-1, Math.min(1, Math.abs(dot))));
    }
}
