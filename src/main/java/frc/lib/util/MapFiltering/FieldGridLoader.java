package frc.lib.util.MapFiltering;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * Utility class for loading a GridMap from a JSON file.
 */
public class FieldGridLoader {

    /** Internal DTO for JSON parsing */
    private static class FieldGridJson {
        public FieldSize field_size;
        public double nodeSizeMeters;
        public boolean[][] grid;
    }

    private static class FieldSize {
        public double x;
        public double y;
    }

    /**
     * Loads the field grid from the deploy/MapFiltering directory.
     *
     * @param fileName JSON file name (e.g. "FieldGrid.json")
     * @return initialized GridMap
     */
    public static GridMap load(String fileName) {
        try {
            Path path = Filesystem.getDeployDirectory()
                    .toPath()
                    .resolve("MapFiltering")
                    .resolve(fileName);

            String json = Files.readString(path);

            ObjectMapper mapper = new ObjectMapper();
            FieldGridJson data = mapper.readValue(json, FieldGridJson.class);

            return new GridMap(
                data.grid,
                data.nodeSizeMeters,
                data.field_size.x,
                data.field_size.y
            );

        } catch (IOException e) {
            throw new RuntimeException("Failed to load FieldGrid JSON", e);
        }
    }
}
