package xpbd.loader;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

public final class ModelLoader {
    private ModelLoader() {
    }

    public static BedrockModelData.Geometry load(String filePath) throws IOException {
        try (BufferedReader reader = new BufferedReader(
                new InputStreamReader(new FileInputStream(filePath), StandardCharsets.UTF_8))) {
            JsonObject root = JsonParser.parseReader(reader).getAsJsonObject();

            if (!root.has("minecraft:geometry")) {
                throw new IOException("Not a valid Bedrock model: missing 'minecraft:geometry'");
            }

            BedrockModelData.GeometryRoot geoRoot = BedrockModelData.GeometryRoot.fromJson(root);
            if (geoRoot.minecraftGeometry.isEmpty()) {
                throw new IOException("No geometry found in model file");
            }
            BedrockModelData.Geometry geo = geoRoot.minecraftGeometry.get(0);
            validateGeometry(geo);
            return geo;
        } catch (RuntimeException e) {
            throw new IOException("Invalid model JSON: " + e.getMessage(), e);
        }
    }

    private static void validateGeometry(BedrockModelData.Geometry geo) throws IOException {
        if (geo == null || geo.bones == null) {
            throw new IOException("Geometry has no bone list");
        }
        Map<String, BedrockModelData.Bone> byName = new LinkedHashMap<>();
        for (BedrockModelData.Bone bone : geo.bones) {
            if (bone == null || bone.name == null || bone.name.isBlank()) {
                throw new IOException("Geometry contains a bone without a name");
            }
            if (byName.putIfAbsent(bone.name, bone) != null) {
                throw new IOException("Duplicate bone name: " + bone.name);
            }
        }
        for (BedrockModelData.Bone bone : geo.bones) {
            if (bone.parent != null && !byName.containsKey(bone.parent)) {
                throw new IOException("Missing parent '" + bone.parent
                        + "' for bone: " + bone.name);
            }
            Set<String> visited = new HashSet<>();
            BedrockModelData.Bone current = bone;
            while (current != null && current.parent != null) {
                if (!visited.add(current.name)) {
                    throw new IOException("Cyclic bone hierarchy at: " + bone.name);
                }
                current = byName.get(current.parent);
            }
        }
    }

    public static BedrockModelData.Bone findBoneByName(List<BedrockModelData.Bone> bones, String name) {
        if (bones == null) return null;
        for (BedrockModelData.Bone b : bones) {
            if (b != null && b.name != null && b.name.equals(name)) return b;
        }
        return null;
    }
}
