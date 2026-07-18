package xpbd.export;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import xpbd.baker.PhysicsBaker;

import java.io.IOException;
import java.util.List;
import java.util.Objects;

/**
 * 将 XPBD 速度数据写入附属文件。Bedrock/YSM 动画 JSON 没有标准速度通道，
 * 因此该文件刻意与动画导出结果分离。
 */
public final class VelocityCacheExporter {
    private VelocityCacheExporter() {
    }

    public static void export(String animationId,
                              List<PhysicsBaker.BakedFrame> frames,
                              double dt,
                              String filePath) throws IOException {
        if (!Double.isFinite(dt) || dt <= 0) {
            throw new IllegalArgumentException("dt must be a finite value greater than 0");
        }
        if (animationId == null || animationId.isBlank()) {
            throw new IllegalArgumentException("animation ID must not be blank");
        }
        Objects.requireNonNull(frames, "frames");

        JsonObject root = new JsonObject();
        root.addProperty("format_version", "1.0.0");
        root.addProperty("cache_type", "xpbd_bone_velocity");
        root.addProperty("animation", animationId);
        root.addProperty("frame_rate", roundTo4(1.0 / dt));
        root.addProperty("space", "model");
        root.addProperty("units", "model_units_per_second");

        JsonArray frameArray = new JsonArray();
        for (PhysicsBaker.BakedFrame frame : frames) {
            JsonObject frameObject = new JsonObject();
            frameObject.addProperty("time", roundTo4(frame.time));

            JsonObject bones = new JsonObject();
            for (PhysicsBaker.BoneState state : frame.boneStates) {
                JsonObject bone = new JsonObject();
                bone.add("linear_velocity", vector(state.linearVelocity));
                bones.add(state.boneName, bone);
            }
            frameObject.add("bones", bones);
            frameArray.add(frameObject);
        }
        root.add("frames", frameArray);

        Gson gson = new GsonBuilder().setPrettyPrinting().create();
        AtomicFileWriter.writeUtf8(filePath, gson.toJson(root));
    }

    private static JsonArray vector(double[] value) {
        double[] safe = value != null ? value : new double[]{0, 0, 0};
        JsonArray result = new JsonArray();
        for (int i = 0; i < 3; i++) {
            double component = i < safe.length ? safe[i] : 0.0;
            if (!Double.isFinite(component)) {
                throw new IllegalArgumentException("velocity components must be finite");
            }
            result.add(roundTo4(component));
        }
        return result;
    }

    private static double roundTo4(double value) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException("velocity cache values must be finite");
        }
        return Math.round(value * 10000.0) / 10000.0;
    }
}
