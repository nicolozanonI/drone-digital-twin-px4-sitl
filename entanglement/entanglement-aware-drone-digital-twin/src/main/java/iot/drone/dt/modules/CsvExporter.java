package iot.drone.dt.modules;

import java.io.BufferedWriter;
import java.io.Closeable;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.Locale;

public class CsvExporter implements Closeable {

    private final BufferedWriter writer;
    private int rowsWritten = 0;

    public CsvExporter(String filePath) throws IOException {
        Path path = Path.of(filePath);

        Path parent = path.getParent();
        if (parent != null) {
            Files.createDirectories(parent);
        }

        this.writer = Files.newBufferedWriter(
                path,
                StandardOpenOption.CREATE,
                StandardOpenOption.TRUNCATE_EXISTING,
                StandardOpenOption.WRITE
        );

        writer.write(
                "timestampUs,droneId," +
                        "posX,posY,posZ," +
                        "velX,velY,velZ," +
                        "odte,timeliness,reliability,packetValidity,packetOrder"
        );
        writer.newLine();
    }

    public synchronized void log(
            long timestampUs,
            String droneId,
            double posX,
            double posY,
            double posZ,
            double velX,
            double velY,
            double velZ,
            double odte,
            double timeliness,
            double reliability,
            double packetValidity,
            double packetOrder
    ) throws IOException {

        String line = String.format(
                Locale.US,
                "%d,%s,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f",
                timestampUs,
                escapeCsv(droneId),
                posX,
                posY,
                posZ,
                velX,
                velY,
                velZ,
                odte,
                timeliness,
                reliability,
                packetValidity,
                packetOrder
        );

        writer.write(line);
        writer.newLine();

        rowsWritten++;

        if (rowsWritten % 100 == 0) {
            writer.flush();
        }
    }

    public synchronized void flush() throws IOException {
        writer.flush();
    }

    @Override
    public synchronized void close() throws IOException {
        writer.flush();
        writer.close();
    }

    private static String escapeCsv(String value) {
        if (value == null) {
            return "";
        }

        if (value.contains(",") || value.contains("\"") || value.contains("\n")) {
            return "\"" + value.replace("\"", "\"\"") + "\"";
        }

        return value;
    }
}
