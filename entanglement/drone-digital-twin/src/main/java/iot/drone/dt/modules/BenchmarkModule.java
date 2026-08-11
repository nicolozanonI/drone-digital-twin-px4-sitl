package iot.drone.dt.modules;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class BenchmarkModule {
    private final ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1);

    public void executeEvery(Runnable task, long rateMs) {
        scheduler.scheduleAtFixedRate(task, 5000, rateMs, TimeUnit.MILLISECONDS);
    }

    public ScheduledExecutorService getScheduler() {
        return this.scheduler;
    }

    public void stop() {
        scheduler.shutdown();
    }

}
