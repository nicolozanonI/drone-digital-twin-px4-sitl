package iot.drone.dt.utils;


import lombok.*;

import java.util.stream.DoubleStream;

import io.vertx.core.json.JsonObject;

@AllArgsConstructor
@NoArgsConstructor
@Getter
@Setter
@ToString
public class Vector3D {

    private double x;
    private double y;
    private double z;

    public boolean isObjectMoving() {
        return DoubleStream.of(x, y, z).anyMatch(value -> Math.abs(value) > 0.15);
    }

    // ==================== OPERATIONS ====================
    public Vector3D add(Vector3D other) {
        return new Vector3D(
                this.x + other.x,
                this.y + other.y,
                this.z + other.z
        );
    }

    public Vector3D subtract(Vector3D other) {
        return new Vector3D(
                this.x - other.x,
                this.y - other.y,
                this.z - other.z
        );
    }

    public Vector3D multiply(double scalar) {
        return new Vector3D(
                this.x * scalar,
                this.y * scalar,
                this.z * scalar
        );
    }

    // ==================== JSON ====================
    public JsonObject getJsonObject() {
        return new JsonObject()
                .put("x", this.x)
                .put("y", this.y)
                .put("z", this.z);
    }
}