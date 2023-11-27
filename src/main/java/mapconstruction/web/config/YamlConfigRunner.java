package mapconstruction.web.config;

import org.yaml.snakeyaml.Yaml;

import java.io.*;

public class YamlConfigRunner {

    public static GeneralConfig getGeneralConfig() throws IOException {
        return getGeneralConfig("general-config.yml");
    }

    public static GeneralConfig getGeneralConfig(String path) throws IOException {
        Yaml yaml = new Yaml();

        try (InputStream in = new FileInputStream(new File(path))) {
            GeneralConfig config = yaml.loadAs(in, GeneralConfig.class);
            System.out.println("Config in " + path + " has the following values:");
            System.out.println(config.toString());
            return config;
        }
    }

    public static void writeGeneralConfigToYaml(GeneralConfig config) {
        try {
            Yaml yaml = new Yaml();
            String output = yaml.dump(config);
            byte[] sourceByte = output.getBytes();
            File file = new File("general-config.yml");
            if (!file.exists()) {
                if (!file.createNewFile()) {
                    System.out.println("Error in writing the config file");
                }
            }
            FileOutputStream fileOutputStream = new FileOutputStream(file);
            fileOutputStream.write(sourceByte);
            fileOutputStream.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static DatasetConfig getDatasetConfig(File path) throws IOException {
        Yaml yaml = new Yaml();

        try (InputStream in = new FileInputStream(path)) {
            return yaml.loadAs(in, DatasetConfig.class);
        }
    }

}