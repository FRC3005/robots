package frc.swervetest;

import java.io.File;

import org.tinylog.configuration.Configuration;

public class LogConfig {
    // Don't show class or function names as this has an impact on performance
    // see https://tinylog.org/v2/benchmark/
    private static final String kFormat = "[UserLog] {date:yyyy-MM-dd HH:mm:ss} - {tag: No Tag} - {level}: {message}";

    public static void config(boolean isSim) {
        String logPath = null;

        if (isSim) {
            logPath = "log.{count}.log";
        } else if (new File("/u").exists()) {
            logPath = "/u/log.{count}.log";
        } else {
            System.out.println("No external drive detected, file logging will be skipped");
        }

        if (logPath != null) {
            Configuration.set("writer1", "rolling file");
            Configuration.set("writer1.file", logPath);
            Configuration.set("writer1.level", "info");
            Configuration.set("writer1.format", kFormat);
            Configuration.set("writer1.buffered", "true");
            Configuration.set("writer1.append", "true");
            Configuration.set("writer1.append", "true");
            Configuration.set("writer1.backups", "100");
        }

        Configuration.set("writer2", "console");
        Configuration.set("writer2.level", "info");
        Configuration.set("writer2.format", kFormat);

        // Always use a separate thread
        Configuration.set("writingthread", "true");
    }
}
