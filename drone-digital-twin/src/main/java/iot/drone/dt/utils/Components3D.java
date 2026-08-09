package iot.drone.dt.utils;


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
        // Controlla se il valore assoluto di almeno una componente supera la soglia
        return DoubleStream.of(x, y, z).anyMatch(value -> Math.abs(value) > 0.15);
    }


    // ==================== OPERAZIONI MATEMATICHE ====================

    /**
     * Somma questo vettore con un altro vettore Components3D.
     * @param other L'altro vettore da sommare.
     * @return Un nuovo oggetto Components3D con la somma delle componenti.
     */
    public Components3D add(Components3D other) {
        return new Components3D(
                this.x + other.x,
                this.y + other.y,
                this.z + other.z
        );
    }

    /**
     * Sottrae un altro vettore da questo vettore.
     * Utile ad esempio per calcolare l'errore tra posizione desiderata e attuale.
     */
    public Components3D subtract(Components3D other) {
        return new Components3D(
                this.x - other.x,
                this.y - other.y,
                this.z - other.z
        );
    }

    /**
     * Moltiplica tutte le componenti per uno scalare.
     */
    public Components3D multiply(double scalar) {
        return new Components3D(
                this.x * scalar,
                this.y * scalar,
                this.z * scalar
        );
    }

}