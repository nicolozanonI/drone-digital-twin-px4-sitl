package iot.drone.dt.utils;


import lombok.*;

import java.util.stream.DoubleStream;

import io.vertx.core.json.JsonObject;
import lombok.*;

import java.util.stream.DoubleStream;

@AllArgsConstructor
@NoArgsConstructor
@Getter
@Setter
@ToString
public class Components3D {

    private double x;
    private double y;
    private double z;

    public boolean isObjectMoving() {
        return DoubleStream.of(x, y, z).anyMatch(value -> Math.abs(value) > 0.15);
    }

    // ==================== OPERAZIONI MATEMATICHE ====================
    public Components3D add(Components3D other) {
        return new Components3D(
                this.x + other.x,
                this.y + other.y,
                this.z + other.z
        );
    }

    public Components3D subtract(Components3D other) {
        return new Components3D(
                this.x - other.x,
                this.y - other.y,
                this.z - other.z
        );
    }

    public Components3D multiply(double scalar) {
        return new Components3D(
                this.x * scalar,
                this.y * scalar,
                this.z * scalar
        );
    }

    // ==================== CONVERSIONE IN JSON ====================
    public JsonObject getJsonObject() {
        return new JsonObject()
                .put("x", this.x)
                .put("y", this.y)
                .put("z", this.z);
    }
}