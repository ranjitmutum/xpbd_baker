package xpbd.loader;

import java.util.Collection;

/** 仅检查当前动画中已选物理骨骼是否包含 Molang 关键帧表达式。 */
public final class MolangKeyframeDetector {
    private MolangKeyframeDetector() {
    }

    public static boolean hasMolangKeyframes(
            BedrockAnimationData.Animation animation,
            Collection<String> selectedBoneIds) {
        if (animation == null || selectedBoneIds == null
                || selectedBoneIds.isEmpty()) {
            return false;
        }
        for (String boneId : selectedBoneIds) {
            if (boneId == null) continue;
            BedrockAnimationData.BoneAnimation bone = animation.bones.get(boneId);
            if (bone != null && (containsMolang(bone.position)
                    || containsMolang(bone.rotation)
                    || containsMolang(bone.scale))) {
                return true;
            }
        }
        return false;
    }

    private static boolean containsMolang(BedrockAnimationData.Keyframes channel) {
        return channel != null && channel.containsMolang();
    }
}
