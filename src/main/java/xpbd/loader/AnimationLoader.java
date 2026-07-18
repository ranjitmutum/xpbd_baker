package xpbd.loader;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;

public final class AnimationLoader {
    private AnimationLoader() {
    }

    public static BedrockAnimationData.AnimationRoot load(String filePath) throws IOException {
        try (BufferedReader reader = new BufferedReader(
                new InputStreamReader(new FileInputStream(filePath), StandardCharsets.UTF_8))) {
            JsonElement parsed = JsonParser.parseReader(reader);
            if (!parsed.isJsonObject()) {
                throw new IOException("Animation JSON root must be an object");
            }
            JsonObject root = parsed.getAsJsonObject();
            if (!root.has("animations") || !root.get("animations").isJsonObject()) {
                throw new IOException("Not a valid Bedrock animation: missing 'animations' object");
            }
            return BedrockAnimationData.AnimationRoot.fromJson(root);
        } catch (RuntimeException e) {
            throw new IOException("Invalid animation JSON: " + e.getMessage(), e);
        }
    }
}
