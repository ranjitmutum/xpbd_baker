package xpbd.export;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import xpbd.baker.PhysicsBaker;
import xpbd.loader.BedrockAnimationData;

import java.io.IOException;
import java.math.BigDecimal;
import java.util.*;
import java.util.function.DoubleUnaryOperator;

public final class AnimationExporter {
    private AnimationExporter() {
    }

    public static void export(String animId, BedrockAnimationData.Animation sourceAnimation,
                              List<PhysicsBaker.BakedFrame> frames,
                              boolean loop, String filePath) throws IOException {
        export(animId, sourceAnimation, frames,
                loop ? BedrockAnimationData.Animation.LoopBehavior.LOOP
                        : BedrockAnimationData.Animation.LoopBehavior.ONCE,
                filePath);
    }

    public static void export(String animId, BedrockAnimationData.Animation sourceAnimation,
                              List<PhysicsBaker.BakedFrame> frames,
                              BedrockAnimationData.Animation.LoopBehavior loopBehavior,
                              String filePath) throws IOException {
        export(animId, sourceAnimation, frames, loopBehavior, filePath,
                null, false);
    }

    public static void export(String animId,
                              BedrockAnimationData.Animation referenceAnimation,
                              List<PhysicsBaker.BakedFrame> frames,
                              BedrockAnimationData.Animation.LoopBehavior loopBehavior,
                              String filePath,
                              DoubleUnaryOperator referenceTimeMapper,
                              boolean exactBakedLength) throws IOException {
        if (animId == null || animId.isBlank()) {
            throw new IllegalArgumentException("animation ID must not be blank");
        }
        Objects.requireNonNull(frames, "frames");

        JsonObject root = new JsonObject();
        root.addProperty("format_version", "1.8.0");

        JsonObject animations = new JsonObject();
        JsonObject anim = new JsonObject();

        BedrockAnimationData.Animation.LoopBehavior safeLoop = loopBehavior == null
                ? BedrockAnimationData.Animation.LoopBehavior.ONCE : loopBehavior;
        if (safeLoop == BedrockAnimationData.Animation.LoopBehavior.HOLD_LAST) {
            anim.addProperty("loop", "hold_on_last_frame");
        } else {
            anim.addProperty("loop",
                    safeLoop == BedrockAnimationData.Animation.LoopBehavior.LOOP);
        }
        if (referenceAnimation != null
                && referenceAnimation.overridePreviousAnimation != null) {
            anim.addProperty("override_previous_animation",
                    referenceAnimation.overridePreviousAnimation);
        }

        double animLen = frames.isEmpty() ? 0.0 : frames.get(frames.size() - 1).time;
        if (!exactBakedLength && referenceAnimation != null
                && referenceAnimation.animationLength > animLen) {
            animLen = referenceAnimation.animationLength;
        }
        requireFinite(animLen, "animation length");
        anim.addProperty("animation_length", animLen);

        JsonObject bones = new JsonObject();

        // 收集所有骨骼名称。
        LinkedHashSet<String> allBoneNames = new LinkedHashSet<>();
        if (referenceAnimation != null) {
            allBoneNames.addAll(referenceAnimation.bones.keySet());
        }
        Set<String> bakedBoneNames = new LinkedHashSet<>();
        for (PhysicsBaker.BakedFrame f : frames) {
            for (PhysicsBaker.BoneState bs : f.boneStates) {
                bakedBoneNames.add(bs.boneName);
                allBoneNames.add(bs.boneName);
            }
        }

        for (String boneName : allBoneNames) {
            JsonObject boneObj = new JsonObject();
            boolean hasBaked = bakedBoneNames.contains(boneName);
            BedrockAnimationData.BoneAnimation srcBone = (referenceAnimation != null)
                    ? referenceAnimation.bones.get(boneName) : null;

            if (hasBaked) {
                JsonObject rotObj = new JsonObject();
                JsonObject posObj = new JsonObject();
                for (PhysicsBaker.BakedFrame f : frames) {
                    PhysicsBaker.BoneState bs = findBoneState(f, boneName);
                    if (bs != null) {
                        double[] rot = bs.rotation != null ? bs.rotation : new double[]{0, 0, 0};
                        double[] pos = bs.position != null ? bs.position : new double[]{0, 0, 0};
                        rotObj.add(fmt(f.time), toArray(rot));
                        posObj.add(fmt(f.time), toArray(pos));
                    }
                }
        // 位置通道与预览使用相同的烘焙局部通道，以保留源动作并支持片段首尾混合。
                boneObj.add("position", posObj);
                boneObj.add("rotation", rotObj);
            } else {
        // 独立过渡会将 B 重采样至过渡时间线；普通烘焙原样保留源关键帧。
                if (srcBone != null && srcBone.position != null) {
                    boneObj.add("position", referenceTimeMapper == null
                            ? keyframesToJson(srcBone.position)
                            : sampledChannelToJson(srcBone.position, frames,
                            referenceTimeMapper));
                }
                if (srcBone != null && srcBone.rotation != null) {
                    boneObj.add("rotation", referenceTimeMapper == null
                            ? keyframesToJson(srcBone.rotation)
                            : sampledChannelToJson(srcBone.rotation, frames,
                            referenceTimeMapper));
                }
            }

        // 缩放与非物理通道采用相同的参考时间线。
            if (srcBone != null && srcBone.scale != null) {
                boneObj.add("scale", referenceTimeMapper == null
                        ? keyframesToJson(srcBone.scale)
                        : sampledChannelToJson(srcBone.scale, frames,
                        referenceTimeMapper));
            }

            bones.add(boneName, boneObj);
        }

        anim.add("bones", bones);
        animations.add(animId, anim);
        root.add("animations", animations);

        Gson gson = new GsonBuilder().setPrettyPrinting().create();
        AtomicFileWriter.writeUtf8(filePath, gson.toJson(root));
    }

    private static JsonElement keyframesToJson(BedrockAnimationData.Keyframes kf) {
        JsonElement originalMolangJson = kf.originalMolangJson();
        if (originalMolangJson != null) return originalMolangJson;
        JsonObject obj = new JsonObject();
        for (Map.Entry<Double, double[]> entry : kf.keyframes.entrySet()) {
            BedrockAnimationData.Keyframes.InterpolationMode interpolationMode =
                    kf.interpolationMode(entry.getKey());
            if (kf.hasDistinctPrePost(entry.getKey())
                    || interpolationMode != BedrockAnimationData.Keyframes
                    .InterpolationMode.LINEAR) {
                JsonObject split = new JsonObject();
                if (kf.hasDistinctPrePost(entry.getKey())) {
                    split.add("pre", toArray(kf.preValue(entry.getKey())));
                }
                split.add("post", toArray(entry.getValue()));
                if (interpolationMode != BedrockAnimationData.Keyframes
                        .InterpolationMode.LINEAR) {
                    split.addProperty("lerp_mode", interpolationMode.jsonName());
                }
                obj.add(fmt(entry.getKey()), split);
            } else {
                obj.add(fmt(entry.getKey()), toArray(entry.getValue()));
            }
        }
        return obj;
    }

    private static JsonObject sampledChannelToJson(
            BedrockAnimationData.Keyframes keyframes,
            List<PhysicsBaker.BakedFrame> frames,
            DoubleUnaryOperator referenceTimeMapper) {
        JsonObject result = new JsonObject();
        for (PhysicsBaker.BakedFrame frame : frames) {
            double referenceTime = referenceTimeMapper.applyAsDouble(frame.time);
            requireFinite(referenceTime, "reference animation time");
            result.add(fmt(frame.time), toArray(keyframes.evaluate(referenceTime)));
        }
        return result;
    }

    private static JsonArray toArray(double[] v) {
        if (v == null || v.length < 3) {
            throw new IllegalArgumentException("animation vectors must contain three components");
        }
        JsonArray arr = new JsonArray();
        arr.add(roundTo4(v[0]));
        arr.add(roundTo4(v[1]));
        arr.add(roundTo4(v[2]));
        return arr;
    }

    private static PhysicsBaker.BoneState findBoneState(PhysicsBaker.BakedFrame frame, String name) {
        return frame.getBoneState(name);
    }

    private static String fmt(double v) {
        requireFinite(v, "keyframe time");
        BigDecimal decimal = BigDecimal.valueOf(v).stripTrailingZeros();
        return decimal.setScale(Math.max(4, decimal.scale())).toPlainString();
    }

    private static double roundTo4(double v) {
        requireFinite(v, "animation value");
        return Math.round(v * 10000.0) / 10000.0;
    }

    private static void requireFinite(double value, String label) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(label + " must be finite");
        }
    }
}
