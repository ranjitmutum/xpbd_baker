package xpbd.loader;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import java.util.ArrayList;
import java.util.List;

public final class BedrockModelData {
    private BedrockModelData() {
    }

    public static class GeometryRoot {
        public List<Geometry> minecraftGeometry;

        public static GeometryRoot fromJson(JsonObject json) {
            GeometryRoot root = new GeometryRoot();
            root.minecraftGeometry = new ArrayList<>();
            JsonArray arr = json.getAsJsonArray("minecraft:geometry");
            for (JsonElement elem : arr) {
                root.minecraftGeometry.add(Geometry.fromJson(elem.getAsJsonObject()));
            }
            return root;
        }
    }

    public static class Geometry {
        public Description description;
        public List<Bone> bones = new ArrayList<>();

        public static Geometry fromJson(JsonObject json) {
            Geometry g = new Geometry();
            g.description = Description.fromJson(json.getAsJsonObject("description"));
            if (json.has("bones")) {
                JsonArray bonesArr = json.getAsJsonArray("bones");
                for (JsonElement elem : bonesArr) {
                    g.bones.add(Bone.fromJson(elem.getAsJsonObject()));
                }
            }
            return g;
        }
    }

    public static class Description {
        public String identifier = "unknown";

        public static Description fromJson(JsonObject json) {
            Description d = new Description();
            if (json.has("identifier")) {
                d.identifier = json.get("identifier").getAsString();
            }
            return d;
        }
    }

    public static class Bone {
        public String name;
        public String parent;
        public double[] pivot = {0, 0, 0};
        public double[] rotation = {0, 0, 0};
        public List<Cube> cubes = new ArrayList<>();
        public static Bone fromJson(JsonObject json) {
            Bone b = new Bone();
            b.name = json.get("name").getAsString();
            if (json.has("parent")) {
                b.parent = json.get("parent").getAsString();
            }
            if (json.has("pivot")) {
                JsonArray piv = json.getAsJsonArray("pivot");
                b.pivot = vector3(piv, "bone pivot");
            }
            if (json.has("rotation")) {
                JsonArray rot = json.getAsJsonArray("rotation");
                b.rotation = vector3(rot, "bone rotation");
            }
            if (json.has("cubes")) {
                JsonArray cubesArr = json.getAsJsonArray("cubes");
                for (JsonElement elem : cubesArr) {
                    b.cubes.add(Cube.fromJson(elem.getAsJsonObject()));
                }
            }
            return b;
        }

    }

    public static class Cube {
        public double[] origin = {0, 0, 0};
        public double[] size = {1, 1, 1};
        public double[] pivot = null;
        public double[] rotation = null;
        public double inflate = 0;

        public static Cube fromJson(JsonObject json) {
            Cube c = new Cube();
            if (!json.has("origin")) {
                throw new IllegalArgumentException("cube origin is required");
            }
            JsonArray o = json.getAsJsonArray("origin");
            c.origin = vector3(o, "cube origin");
            if (!json.has("size")) {
                throw new IllegalArgumentException("cube size is required");
            }
            JsonArray s = json.getAsJsonArray("size");
            c.size = vector3(s, "cube size");
            if (json.has("pivot")) {
                JsonArray p = json.getAsJsonArray("pivot");
                c.pivot = vector3(p, "cube pivot");
            }
            if (json.has("rotation")) {
                JsonArray r = json.getAsJsonArray("rotation");
                c.rotation = vector3(r, "cube rotation");
            }
            if (json.has("inflate")) {
                c.inflate = json.get("inflate").getAsDouble();
                if (!Double.isFinite(c.inflate)) {
                    throw new IllegalArgumentException("cube inflate must be finite");
                }
            }
            for (double component : c.size) {
                if (Math.abs(component) + c.inflate * 2.0 < 0) {
                    throw new IllegalArgumentException(
                            "cube inflate shrinks an effective size below zero");
                }
            }
            return c;
        }
    }

    private static double[] vector3(JsonArray array, String label) {
        if (array == null || array.size() < 3) {
            throw new IllegalArgumentException(label + " must contain three numbers");
        }
        double[] result = new double[3];
        for (int i = 0; i < result.length; i++) {
            result[i] = array.get(i).getAsDouble();
            if (!Double.isFinite(result[i])) {
                throw new IllegalArgumentException(label + " components must be finite");
            }
        }
        return result;
    }
}
