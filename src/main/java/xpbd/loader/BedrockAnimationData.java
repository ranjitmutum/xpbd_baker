package xpbd.loader;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.TreeMap;

public final class BedrockAnimationData {
    private BedrockAnimationData() {
    }

    public static class AnimationRoot {
        public Map<String, Animation> animations = new LinkedHashMap<>();

        public static AnimationRoot fromJson(com.google.gson.JsonObject json) {
            AnimationRoot root = new AnimationRoot();
            if (!json.has("format_version")) {
                throw new IllegalArgumentException("missing format_version");
            }
            json.get("format_version").getAsString();
            if (!json.has("animations") || !json.get("animations").isJsonObject()) {
                throw new IllegalArgumentException("missing animations object");
            }
            for (Map.Entry<String, com.google.gson.JsonElement> entry :
                    json.getAsJsonObject("animations").entrySet()) {
                root.animations.put(entry.getKey(), Animation.fromJson(
                        entry.getKey(), entry.getValue().getAsJsonObject()));
            }
            return root;
        }
    }

    public static class Animation {
        public enum LoopBehavior {
            ONCE,
            LOOP,
            HOLD_LAST
        }

        public boolean loop;
        public LoopBehavior loopBehavior = LoopBehavior.ONCE;
        public double animationLength;
        public Boolean overridePreviousAnimation;
        public Map<String, BoneAnimation> bones = new LinkedHashMap<>();

        public static Animation fromJson(com.google.gson.JsonObject json) {
            return fromJson("<animation>", json);
        }

        private static Animation fromJson(String animationName,
                                          com.google.gson.JsonObject json) {
            Animation a = new Animation();
            if (json.has("loop")) {
                com.google.gson.JsonElement loopElem = json.get("loop");
                if (loopElem.isJsonPrimitive() && loopElem.getAsJsonPrimitive().isBoolean()) {
                    a.loop = loopElem.getAsBoolean();
                    a.loopBehavior = a.loop ? LoopBehavior.LOOP : LoopBehavior.ONCE;
                } else if (loopElem.isJsonPrimitive()
                        && loopElem.getAsJsonPrimitive().isString()
                        && "hold_on_last_frame".equals(loopElem.getAsString())) {
                    a.loop = false;
                    a.loopBehavior = LoopBehavior.HOLD_LAST;
                } else {
                    throw new IllegalArgumentException(
                            "loop must be boolean or 'hold_on_last_frame'");
                }
            }
            if (json.has("animation_length")) {
                a.animationLength = json.get("animation_length").getAsDouble();
            } else {
                a.animationLength = Double.NaN;
            }
            if (json.has("override_previous_animation")) {
                com.google.gson.JsonElement override = json.get("override_previous_animation");
                if (!override.isJsonPrimitive()
                        || !override.getAsJsonPrimitive().isBoolean()) {
                    throw new IllegalArgumentException(
                            "override_previous_animation must be boolean");
                }
                a.overridePreviousAnimation = override.getAsBoolean();
            }
            if (json.has("bones")) {
                for (Map.Entry<String, com.google.gson.JsonElement> entry :
                        json.getAsJsonObject("bones").entrySet()) {
                    BoneAnimation boneAnimation = BoneAnimation.fromJson(
                            animationName, entry.getKey(),
                            entry.getValue().getAsJsonObject());
                    boneAnimation.setLooping(a.loop);
                    a.bones.put(entry.getKey(), boneAnimation);
                }
            }
            if (Double.isNaN(a.animationLength)) {
                a.animationLength = maximumKeyframeTime(a);
            } else if (!Double.isFinite(a.animationLength) || a.animationLength < 0) {
                throw new IllegalArgumentException(
                        "animation_length must be a finite non-negative number");
            }
            return a;
        }

        private static double maximumKeyframeTime(Animation animation) {
            double maximum = 0;
            for (BoneAnimation bone : animation.bones.values()) {
                if (bone == null) continue;
                maximum = Math.max(maximum, maximumKeyframeTime(bone.position));
                maximum = Math.max(maximum, maximumKeyframeTime(bone.rotation));
                maximum = Math.max(maximum, maximumKeyframeTime(bone.scale));
            }
            return maximum;
        }

        private static double maximumKeyframeTime(Keyframes channel) {
            if (channel == null || channel.keyframes.isEmpty()) return 0;
            double maximum = 0;
            for (Double time : channel.keyframes.keySet()) {
                if (time != null) maximum = Math.max(maximum, time);
            }
            return maximum;
        }
    }

    public static class BoneAnimation {
        public Keyframes position;
        public Keyframes rotation;
        public Keyframes scale;

        public static BoneAnimation fromJson(com.google.gson.JsonObject json) {
            return fromJson("<animation>", "<bone>", json);
        }

        private static BoneAnimation fromJson(String animationName, String boneName,
                                              com.google.gson.JsonObject json) {
            BoneAnimation ba = new BoneAnimation();
            if (json.has("position")) {
                ba.position = Keyframes.fromJson(json.get("position"),
                        context(animationName, boneName, "position"));
            }
            if (json.has("rotation")) {
                ba.rotation = Keyframes.fromJson(json.get("rotation"),
                        context(animationName, boneName, "rotation"));
            }
            if (json.has("scale")) {
                ba.scale = Keyframes.fromJson(json.get("scale"),
                        context(animationName, boneName, "scale"));
            }
            return ba;
        }

        private static String context(String animationName, String boneName,
                                      String channel) {
            return "animation '" + animationName + "', bone '" + boneName
                    + "', channel '" + channel + "'";
        }

        private void setLooping(boolean looping) {
            if (position != null) position.setLooping(looping);
            if (rotation != null) rotation.setLooping(looping);
            if (scale != null) scale.setLooping(looping);
        }
    }

    public static class Keyframes {
        public enum InterpolationMode {
            LINEAR("linear"),
            CATMULLROM("catmullrom");

            private final String jsonName;

            InterpolationMode(String jsonName) {
                this.jsonName = jsonName;
            }

            public String jsonName() {
                return jsonName;
            }
        }

            // JSON 对象属性顺序不能代表时间线顺序；排序映射可处理乱序的时间键。
        public Map<Double, double[]> keyframes = new TreeMap<>();
        private final Map<Double, double[]> preKeyframes = new TreeMap<>();
        private final Map<Double, InterpolationMode> interpolationModes = new TreeMap<>();
        private boolean containsMolang;
        private com.google.gson.JsonElement originalMolangJson;
        private boolean looping;

        public static Keyframes fromJson(com.google.gson.JsonElement elem) {
            return fromJson(elem, "animation keyframe");
        }

        private static Keyframes fromJson(com.google.gson.JsonElement elem,
                                          String context) {
            if (elem == null || elem.isJsonNull()) {
                throw new IllegalArgumentException(context + ": keyframe data is missing");
            }
            Keyframes kf = new Keyframes();
            kf.containsMolang = isMolangValue(elem);
            if (kf.containsMolang) {
                kf.originalMolangJson = elem.deepCopy();
            }
            if (elem.isJsonPrimitive()) {
                if (elem.getAsJsonPrimitive().isNumber()) {
                    double v = requireFinite(elem.getAsDouble(),
                            context + " at time 0 component");
                    kf.put(0.0, new double[]{v, v, v}, new double[]{v, v, v},
                            InterpolationMode.LINEAR);
                } else if (isMolangValue(elem)) {
                    double[] value = new double[]{0, 0, 0};
                    kf.put(0.0, value, value, InterpolationMode.LINEAR);
                } else {
                    throw new IllegalArgumentException(
                            context + " at time 0: scalar must be numeric");
                }
            } else if (elem.isJsonArray()) {
                com.google.gson.JsonArray arr = elem.getAsJsonArray();
                if (!arr.isEmpty() && arr.get(0).isJsonPrimitive()) {
                    double[] value = parseVector(arr, context + " at time 0");
                    kf.put(0.0, value, value, InterpolationMode.LINEAR);
                } else {
                    for (com.google.gson.JsonElement item : arr) {
                        if (!item.isJsonObject()) {
                            throw new IllegalArgumentException(
                                    context + ": timed keyframes must be objects");
                        }
                        com.google.gson.JsonObject obj = item.getAsJsonObject();
                        double time = requireTime(obj.get("time"), context);
                        ParsedValue value = parseKeyframeValue(obj.get("data"),
                                context + " at time " + time);
                        kf.put(time, value.pre, value.post, value.interpolationMode);
                    }
                }
            } else if (elem.isJsonObject()) {
                com.google.gson.JsonObject object = elem.getAsJsonObject();
                if (object.has("pre") || object.has("post")
                        || object.has("lerp_mode")) {
                    ParsedValue value = parseKeyframeValue(object,
                            context + " at time 0");
                    kf.put(0.0, value.pre, value.post, value.interpolationMode);
                } else {
                    for (Map.Entry<String, com.google.gson.JsonElement> entry :
                            object.entrySet()) {
                        double time;
                        try {
                            time = Double.parseDouble(entry.getKey());
                        } catch (NumberFormatException error) {
                            throw new IllegalArgumentException(context
                                    + ": invalid keyframe time '" + entry.getKey() + "'", error);
                        }
                        requireTime(time, context);
                        ParsedValue value = parseKeyframeValue(entry.getValue(),
                                context + " at time " + entry.getKey());
                        kf.put(time, value.pre, value.post, value.interpolationMode);
                    }
                }
            } else {
                throw new IllegalArgumentException(context + ": unsupported keyframe data");
            }
            return kf;
        }

        private static double[] parseVector(com.google.gson.JsonArray arr,
                                            String context) {
            if (arr.size() != 3) {
                throw new IllegalArgumentException(
                        context + ": vector must contain exactly three components");
            }
            double[] result = new double[3];
            for (int i = 0; i < 3; i++) {
                com.google.gson.JsonElement e = arr.get(i);
                if (e.isJsonPrimitive() && e.getAsJsonPrimitive().isNumber()) {
                    result[i] = requireFinite(e.getAsDouble(),
                            context + " component " + i);
                } else if (e.isJsonPrimitive() && e.getAsJsonPrimitive().isString()) {
                    try {
                        result[i] = requireFinite(Double.parseDouble(e.getAsString()),
                                context + " component " + i);
                    } catch (NumberFormatException ex) {
                        if (isMolangValue(e)) {
            // Molang 求值刻意不在烘焙器内完成；零采样可使预览/模拟数据保持数值，
            // 直到用户确认允许烘焙结果替换它。
                            result[i] = 0;
                        } else {
                            throw new IllegalArgumentException(context + " component " + i
                                    + " must be numeric", ex);
                        }
                    }
                } else {
                    throw new IllegalArgumentException(context + " component " + i
                            + " must be numeric");
                }
            }
            return result;
        }

        private static ParsedValue parseKeyframeValue(
                com.google.gson.JsonElement elem, String context) {
            if (elem == null || elem.isJsonNull()) {
                throw new IllegalArgumentException(context + ": keyframe value is missing");
            }
            if (elem.isJsonArray()) {
                double[] value = parseVector(elem.getAsJsonArray(), context);
                return new ParsedValue(value, value, InterpolationMode.LINEAR);
            } else if (elem.isJsonObject()) {
                com.google.gson.JsonObject obj = elem.getAsJsonObject();
                InterpolationMode interpolationMode = parseInterpolationMode(obj, context);
                double[] pre = obj.has("pre")
                        ? parseVectorValue(obj.get("pre"), context + " pre") : null;
                double[] post = obj.has("post")
                        ? parseVectorValue(obj.get("post"), context + " post") : null;
                if (pre == null && post == null) {
                    throw new IllegalArgumentException(
                            context + ": object keyframe needs pre or post");
                }
                if (pre == null) pre = post.clone();
                if (post == null) post = pre.clone();
                return new ParsedValue(pre, post, interpolationMode);
            }
            if (elem.isJsonPrimitive()) {
                double scalar;
                try {
                    scalar = elem.getAsDouble();
                } catch (RuntimeException error) {
                    if (isMolangValue(elem)) {
                        scalar = 0;
                    } else {
                        throw new IllegalArgumentException(
                                context + ": scalar must be numeric", error);
                    }
                }
                requireFinite(scalar, context + " component");
                double[] value = new double[]{scalar, scalar, scalar};
                return new ParsedValue(value, value, InterpolationMode.LINEAR);
            }
            throw new IllegalArgumentException(context + ": unsupported keyframe value");
        }

        private static double[] parseVectorValue(com.google.gson.JsonElement elem,
                                                 String context) {
            if (elem != null && elem.isJsonArray()) {
                return parseVector(elem.getAsJsonArray(), context);
            }
            ParsedValue scalar = parseKeyframeValue(elem, context);
            if (!java.util.Arrays.equals(scalar.pre, scalar.post)) {
                throw new IllegalArgumentException(
                        context + ": nested pre/post keyframes are not supported");
            }
            return scalar.post;
        }

        private static InterpolationMode parseInterpolationMode(
                com.google.gson.JsonObject object, String context) {
            if (!object.has("lerp_mode")) return InterpolationMode.LINEAR;
            com.google.gson.JsonElement element = object.get("lerp_mode");
            if (!element.isJsonPrimitive()
                    || !element.getAsJsonPrimitive().isString()) {
                throw new IllegalArgumentException(
                        context + ": interpolation mode must be a string");
            }
            String mode = element.getAsString();
            if ("linear".equalsIgnoreCase(mode)) return InterpolationMode.LINEAR;
            if ("catmullrom".equalsIgnoreCase(mode)) return InterpolationMode.CATMULLROM;
            throw new IllegalArgumentException(
                    context + ": unsupported interpolation mode '" + mode + "'");
        }

        private void put(double time, double[] pre, double[] post,
                         InterpolationMode interpolationMode) {
            preKeyframes.put(time, pre.clone());
            keyframes.put(time, post.clone());
            interpolationModes.put(time, interpolationMode);
        }

        public double[] evaluate(double time) {
            if (keyframes.isEmpty()) return new double[]{0, 0, 0};
            double queryTime = Double.isFinite(time) ? Math.max(0, time) : 0;
            java.util.NavigableMap<Double, double[]> sorted = keyframes instanceof java.util.NavigableMap
                    ? (java.util.NavigableMap<Double, double[]>) keyframes
                    : new TreeMap<>(keyframes);
            Map.Entry<Double, double[]> previous = sorted.floorEntry(queryTime);
            if (previous != null && Math.abs(previous.getKey() - queryTime) <= 1e-12) {
                return previous.getValue().clone();
            }
            Map.Entry<Double, double[]> next = sorted.ceilingEntry(queryTime);
            if (previous == null) return preValue(next.getKey());
            if (next == null) return previous.getValue().clone();
            double t = (queryTime - previous.getKey())
                    / (next.getKey() - previous.getKey());
            double[] from = previous.getValue();
            double[] to = preValue(next.getKey());
            double[] result = new double[3];
            if (interpolationMode(previous.getKey()) == InterpolationMode.CATMULLROM
                    || interpolationMode(next.getKey()) == InterpolationMode.CATMULLROM) {
                Map.Entry<Double, double[]> beforePrevious = sorted.lowerEntry(
                        previous.getKey());
                Map.Entry<Double, double[]> afterNext = sorted.higherEntry(next.getKey());
                if (looping && sorted.size() >= 3) {
                    if (beforePrevious == null) {
                        beforePrevious = entryAt(sorted, sorted.size() - 2);
                    }
                    if (afterNext == null) afterNext = entryAt(sorted, 1);
                }
                double[] outerBefore = beforePrevious != null
                        && !hasDistinctPrePost(previous.getKey())
                        ? beforePrevious.getValue() : from;
                double[] outerAfter = afterNext != null
                        && !hasDistinctPrePost(next.getKey())
                        ? preValue(afterNext.getKey()) : to;
                for (int i = 0; i < 3; i++) {
                    result[i] = catmullRom(t, outerBefore[i], from[i], to[i],
                            outerAfter[i]);
                }
            } else {
                for (int i = 0; i < 3; i++) {
                    result[i] = from[i] + (to[i] - from[i]) * t;
                }
            }
            return result;
        }

        private static Map.Entry<Double, double[]> entryAt(
                java.util.NavigableMap<Double, double[]> values, int index) {
            int current = 0;
            for (Map.Entry<Double, double[]> entry : values.entrySet()) {
                if (current++ == index) return entry;
            }
            return null;
        }

        private static double catmullRom(double t, double p0, double p1,
                                         double p2, double p3) {
            double v0 = (p2 - p0) * 0.5;
            double v1 = (p3 - p1) * 0.5;
            double t2 = t * t;
            double t3 = t2 * t;
            return (2 * p1 - 2 * p2 + v0 + v1) * t3
                    + (-3 * p1 + 3 * p2 - 2 * v0 - v1) * t2
                    + v0 * t + p1;
        }

        public double[] preValue(double time) {
            double[] pre = preKeyframes.get(time);
            if (pre == null) pre = keyframes.get(time);
            return pre == null ? null : pre.clone();
        }

        public boolean hasDistinctPrePost(double time) {
            double[] pre = preKeyframes.get(time);
            double[] post = keyframes.get(time);
            return pre != null && post != null && !java.util.Arrays.equals(pre, post);
        }

        public InterpolationMode interpolationMode(double time) {
            return interpolationModes.getOrDefault(time, InterpolationMode.LINEAR);
        }

        public boolean containsMolang() {
            return containsMolang;
        }

        /**
         * 若通道包含 Molang，则返回未修改的导入通道副本，避免检测或导出改变源数据。
         */
        public com.google.gson.JsonElement originalMolangJson() {
            return originalMolangJson == null ? null : originalMolangJson.deepCopy();
        }

        /**
         * 递归识别 Bedrock 关键帧值中的 Molang。数值字符串仍按数值处理；
         * 时间、插值模式等关键帧元数据不参与表达式检测。
         */
        public static boolean isMolangValue(com.google.gson.JsonElement value) {
            if (value == null || value.isJsonNull()) return false;
            if (value.isJsonArray()) {
                for (com.google.gson.JsonElement item : value.getAsJsonArray()) {
                    if (isMolangValue(item)) return true;
                }
                return false;
            }
            if (value.isJsonObject()) {
                for (Map.Entry<String, com.google.gson.JsonElement> entry
                        : value.getAsJsonObject().entrySet()) {
                    if ("time".equals(entry.getKey())
                            || "lerp_mode".equals(entry.getKey())) {
                        continue;
                    }
                    if (isMolangValue(entry.getValue())) return true;
                }
                return false;
            }
            if (!value.isJsonPrimitive()
                    || !value.getAsJsonPrimitive().isString()) {
                return false;
            }
            String text = value.getAsString().trim();
            if (text.isEmpty()) return false;
            try {
                Double.parseDouble(text);
                return false;
            } catch (NumberFormatException ignored) {
                return true;
            }
        }

        private void setLooping(boolean looping) {
            this.looping = looping;
        }

        private static double requireTime(com.google.gson.JsonElement element,
                                          String context) {
            if (element == null) {
                throw new IllegalArgumentException(context + ": keyframe time is missing");
            }
            return requireTime(element.getAsDouble(), context);
        }

        private static double requireTime(double time, String context) {
            if (!Double.isFinite(time) || time < 0) {
                throw new IllegalArgumentException(
                        context + ": keyframe time must be a finite non-negative number");
            }
            return time;
        }

        private static double requireFinite(double value, String label) {
            if (!Double.isFinite(value)) {
                throw new IllegalArgumentException(label + " must be finite");
            }
            return value;
        }

        private record ParsedValue(double[] pre, double[] post,
                                   InterpolationMode interpolationMode) {
        }
    }
}
