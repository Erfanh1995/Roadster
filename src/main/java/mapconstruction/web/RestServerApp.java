package mapconstruction.web;

import com.google.common.base.Charsets;
import io.dropwizard.Application;
import io.dropwizard.assets.AssetsBundle;
import io.dropwizard.servlets.assets.AssetServlet;
import io.dropwizard.setup.Bootstrap;
import io.dropwizard.setup.Environment;
import mapconstruction.log.Log;
import mapconstruction.log.LogLevel;
import mapconstruction.web.config.GeneralConfig;
import mapconstruction.web.config.YamlConfigRunner;
import org.eclipse.jetty.servlets.CrossOriginFilter;

import javax.servlet.DispatcherType;
import javax.servlet.FilterRegistration;
import javax.servlet.ServletRegistration;
import java.io.IOException;
import java.net.URL;
import java.util.EnumSet;

/**
 * The starting point of our application when we also want to launch our webservice.
 * It also handles  handles quite a lot of data flow.
 *
 * @author Jorrick
 * @since 18/09/2018
 */
public class RestServerApp extends Application<RestServerConfig> {

    public static void main(String[] args) throws Exception {
        new RestServerApp().run(args);
    }

    @Override
    public void initialize(Bootstrap<RestServerConfig> bootstrap) {
        bootstrap.addBundle(new AssetsBundle("/assets", "/", "index.html"));
    }

    @Override
    public void run(RestServerConfig config, Environment env) {
        boolean devMode = true;

        /*
         * Tried to get dynamic loading of values, unfortunate not working very well.
         */
        ServletRegistration.Dynamic dynamic1 = env.admin().addServlet("assets-js",
                new AssetServlet("/assets/js", "/updated-js/", "", Charsets.UTF_8));
        ServletRegistration.Dynamic dynamic2 = env.admin().addServlet("assets-css",
                new AssetServlet("/assets/css", "/updated-css/", "", Charsets.UTF_8));
        dynamic1.addMapping("/updated-js/*");
        dynamic2.addMapping("/updated-css/*");
        if (devMode) {
            dynamic1.setInitParameter("useFileMappedBuffer", "false");
            dynamic2.setInitParameter("useFileMappedBuffer", "false");
        }

        /*
         * Enabling CORS
         * This allows for easy local testing. If the application runs local, it shouldn't bring any risks.
         * If CORS wasn't enabled, you would get permission errors when running local.
         */
        final FilterRegistration.Dynamic cors =
                env.servlets().addFilter("CORS", CrossOriginFilter.class);
        cors.setInitParameter("allowedOrigins", "*");
        cors.setInitParameter("allowedHeaders", "X-Requested-With,Content-Type,Accept,Origin");
        cors.setInitParameter("allowedMethods", "OPTIONS,GET,PUT,POST,DELETE,HEAD");
        cors.addMappingForUrlPatterns(EnumSet.allOf(DispatcherType.class), true, "/*");

        /*
         * Load the general config of our application
         */
        GeneralConfig yamlConfig;
        try {
            yamlConfig = YamlConfigRunner.getGeneralConfig();
        } catch (IOException ex) {
            YamlConfigRunner.writeGeneralConfigToYaml(new GeneralConfig());
            Log.log(LogLevel.INFO, "APIService", "Problem getting the config. New config created. Run again.");
            yamlConfig = new GeneralConfig();
        }

        Controller controller = new Controller(yamlConfig);

        /*
         * If the user specified it wants to open a web page on start, we open this.
         */
        if (yamlConfig.isOpenWebPageOnStart()) {
            try {
                java.awt.Desktop.getDesktop().browse(new URL("http://127.0.0.1:" + yamlConfig.getWebPagePort()).toURI());
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        final APIService API = new APIService(controller, yamlConfig);
        env.jersey().register(API);

    }
}