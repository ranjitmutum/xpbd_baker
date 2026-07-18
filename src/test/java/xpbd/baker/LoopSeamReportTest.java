package xpbd.baker;

import org.junit.jupiter.api.Test;
import xpbd.loader.BedrockAnimationData;
import xpbd.loader.BedrockModelData;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

final class LoopSeamReportTest {
    @Test
    void separatesInheritedDriverJumpFromPhysicsRelativeContinuity() {
        BedrockModelData.Bone root = bone("root", null);
        BedrockModelData.Bone tip = bone("tip", root.name);
        BedrockAnimationData.Animation animation = animation();
        List<PhysicsBaker.BakedFrame> frames = new ArrayList<>();
        for (int i = 0; i <= 10; i++) {
            double time = i / 10.0;
            double rootX = time <= 0.1 ? time * 10
                    : time >= 0.9 ? (1 - time) * 10 : 1;
            frames.add(new PhysicsBaker.BakedFrame(time, List.of(
                    state(root.name, new double[]{rootX, 0, 0}),
                    state(tip.name, new double[]{0, 1, 0}))));
        }

        LoopSeamReport report = LoopSeamReport.measure(frames,
                List.of(root, tip), animation, Set.of(root.name, tip.name),
                Set.of(root.name), false, 0, true, 0);
        BoneMapper.PhysicsGroupConfig config = new BoneMapper.PhysicsGroupConfig();
        config.loopSeamMatchAcceleration = false;

        assertTrue(report.driver().maximumLinearVelocityJump() > 19);
        assertTrue(report.physicsRelative().maximumLinearVelocityJump() < 1e-9);
        assertTrue(report.passes(config),
                "the default policy must not blame physics for a source-driver jump");

        config.loopSeamStrategy = BoneMapper.LoopSeamStrategy.VISUAL_SUBTREE;
        assertFalse(report.passes(config),
                "whole-subtree visual closure must include the root driver");

        LoopSeamReport collisionRejected = LoopSeamReport.measure(frames,
                List.of(root, tip), animation, Set.of(root.name, tip.name),
                Set.of(root.name), false, 0, false, 0.25);
        config.loopSeamStrategy = BoneMapper.LoopSeamStrategy.PHYSICS_RELATIVE;
        assertFalse(collisionRejected.passes(config),
                "a seam candidate that fails collision auditing must never pass");
    }

    private static BedrockAnimationData.Animation animation() {
        BedrockAnimationData.Animation animation = new BedrockAnimationData.Animation();
        animation.animationLength = 1;
        animation.loop = true;
        BedrockAnimationData.BoneAnimation root = new BedrockAnimationData.BoneAnimation();
        root.position = new BedrockAnimationData.Keyframes();
        root.position.keyframes.put(0.0, new double[]{0, 0, 0});
        root.position.keyframes.put(0.1, new double[]{1, 0, 0});
        root.position.keyframes.put(0.9, new double[]{1, 0, 0});
        root.position.keyframes.put(1.0, new double[]{0, 0, 0});
        animation.bones.put("root", root);
        return animation;
    }

    private static BedrockModelData.Bone bone(String name, String parent) {
        BedrockModelData.Bone bone = new BedrockModelData.Bone();
        bone.name = name;
        bone.parent = parent;
        return bone;
    }

    private static PhysicsBaker.BoneState state(String name, double[] position) {
        return new PhysicsBaker.BoneState(name, position, new double[3],
                new double[3], new double[3]);
    }
}
