package iot.drone.dt.modules;

import io.micrometer.core.instrument.Counter;
import io.micrometer.core.instrument.Gauge;
import io.micrometer.core.instrument.Timer;
import io.micrometer.core.instrument.distribution.CountAtBucket;
import io.micrometer.core.instrument.distribution.HistogramSnapshot;
import io.micrometer.prometheusmetrics.PrometheusConfig;
import io.micrometer.prometheusmetrics.PrometheusMeterRegistry;

import java.time.Duration;
import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;


public class OdteManager {

    public PrometheusMeterRegistry prometheusRegistry = new PrometheusMeterRegistry(PrometheusConfig.DEFAULT);

    public String droneId;

    public OdteManager(String droneId) {
        this.droneId = droneId;
    }

    public record odteRatio(SlidingWindowCounter counter, AtomicReference<Double> gaugeValue, Gauge gauge, Gauge ratioGauge) {}

    private static class SlidingWindowCounter {

        //private final PrometheusMeterRegistry registry;
        private final Counter underlyingCounter;
        private final ConcurrentLinkedDeque<Long> timestamps = new ConcurrentLinkedDeque<>();
        private final long windowMillis;


        public SlidingWindowCounter(PrometheusMeterRegistry registry, String name, String description, long windowSeconds) {
            //this.registry = registry;
            this.windowMillis = TimeUnit.SECONDS.toMillis(windowSeconds);
            this.underlyingCounter = Counter.builder(name)
                    .description(description + " (total)")
                    .register(registry);
        }

        public void increment() {
            underlyingCounter.increment(1.0);
            long now = System.currentTimeMillis();
            timestamps.addLast(now);
            long windowStart = now - windowMillis;
            while (!timestamps.isEmpty() && timestamps.peekFirst() < windowStart) {
                timestamps.removeFirst();
            }
        }

        public double getWindowTotalCount() {

            long now = System.currentTimeMillis();
            long windowStart = now - windowMillis;
            while (!timestamps.isEmpty() && timestamps.peekFirst() < windowStart) {
                timestamps.removeFirst();
            }

            return timestamps.size();
        }
    }

    // To compute availability and reliability ratios
    private odteRatio createMetrics(PrometheusMeterRegistry prometheusRegistry,
                                      String counterName, String counterDescription,
                                      String gaugeName, String gaugeDescription,
                                      String ratioGaugeName, String ratioGaugeDescription,
                                      double initialValue) {

        SlidingWindowCounter counter = new SlidingWindowCounter(prometheusRegistry, counterName, counterDescription, 30);
        AtomicReference<Double> expectedGaugeValue = new AtomicReference<>(initialValue);
        Gauge gauge = Gauge.builder(gaugeName, expectedGaugeValue, AtomicReference::get)
                .description(gaugeDescription)
                .register(prometheusRegistry);

        Gauge ratioGauge = Gauge.builder(ratioGaugeName, () -> {

                    double actualUpdatesInWindow = counter.getWindowTotalCount();
                    double expectedRatePerSecond = expectedGaugeValue.get();
                    final double WINDOW_SECONDS = 30.0;
                    double expectedUpdatesInWindow = expectedRatePerSecond * WINDOW_SECONDS;

                    if (expectedUpdatesInWindow == 0.0) {
                        return 0.0;
                    }
                    double ratio = actualUpdatesInWindow / expectedUpdatesInWindow;
                    return Math.min(1.0, Math.max(0.0, ratio));
                })

                .description(ratioGaugeDescription)
                .register(prometheusRegistry);

        return new odteRatio(counter, expectedGaugeValue, gauge, ratioGauge);
    }

    // Reliability Digital Twin to Physical Twin
    public record ReliabilityPhysicalToDigital(
            SlidingWindowCounter executedCounter,
            SlidingWindowCounter requestedCounter,
            Gauge ratioGauge
    ) {}

    // Reliability DT to PT
    private ReliabilityPhysicalToDigital createReliabilityDigitalToPhysicalMetric(
            PrometheusMeterRegistry prometheusRegistry,
            String executedCounterName, String executedDescription,
            String requestedCounterName, String requestedDescription,
            String ratioGaugeName, String ratioGaugeDescription,
            long windowSeconds) {

        // Counter of actions executed by the drone
        SlidingWindowCounter executedCounter = new SlidingWindowCounter(
                prometheusRegistry,
                executedCounterName,
                executedDescription,
                windowSeconds
        );

        // Counter of actions requested by Digital Twin
        SlidingWindowCounter requestedActionsCounter = new SlidingWindowCounter(
                prometheusRegistry,
                requestedCounterName,
                requestedDescription,
                windowSeconds
        );

        Gauge ratioGauge = Gauge.builder(ratioGaugeName, () -> {

                    double executedInWindow = executedCounter.getWindowTotalCount();
                    double requestedInWindow = requestedActionsCounter.getWindowTotalCount();
                    if (requestedInWindow == 0.0) {
                        return 0.0;
                    }
                    double ratio = executedInWindow / requestedInWindow;
                    return Math.min(1.0, Math.max(0.0, ratio));

                })
                .description(ratioGaugeDescription)
                .register(prometheusRegistry);

        return new ReliabilityPhysicalToDigital(executedCounter, requestedActionsCounter, ratioGauge);
    }

    // Timeliness
    public record Timeliness(Timer timer, Gauge ratioGauge, AtomicReference<Duration> sloThreshold
    ) {}

    private Timeliness createTimerWithRatio(
            String timerName, String timerDescription,
            String ratioGaugeName, String ratioGaugeDescription,
            double percentile, Duration expiryDuration,
            Duration initialSloThreshold,
            Duration slidingWindow) {

        Timer timer = Timer.builder(timerName)
                .description(timerDescription)
                .publishPercentiles(percentile)
                .distributionStatisticExpiry(slidingWindow)
                .serviceLevelObjectives(initialSloThreshold)
                .register(prometheusRegistry);

        AtomicReference<Duration> sloThresholdRef = new AtomicReference<>(initialSloThreshold);

        Gauge ratioGauge = Gauge.builder(ratioGaugeName, timer, t -> {
                    HistogramSnapshot snapshot = t.takeSnapshot();
                    long total = snapshot.count();
                    if (total == 0) return 0.0;

                    long countUnderSlo = 0;

                    Duration currentThreshold = sloThresholdRef.get();

                    for (CountAtBucket bucket : snapshot.histogramCounts()) {
                        if (bucket.bucket(TimeUnit.NANOSECONDS) <= currentThreshold.toNanos()) {
                            countUnderSlo = (long) bucket.count();
                        }
                    }

                    double ratio = (double) countUnderSlo / total;
                    return Math.min(1.0, Math.max(0.0, ratio));
                })
                .description(ratioGaugeDescription)
                .register(prometheusRegistry);

        return new Timeliness(timer, ratioGauge, sloThresholdRef);
    }


    public record JitterWithRatio(JitterTracker tracker, Gauge ratioGauge, AtomicReference<Duration> jitterThreshold) {}

    public static class JitterTracker {
        private final ConcurrentLinkedDeque<JitterMeasurement> measurements = new ConcurrentLinkedDeque<>();
        private final long windowMillis;
        private long lastEventTime = -1;

        record JitterMeasurement(long timestamp, long jitterMillis) {}

        public JitterTracker(long windowSeconds) {
            this.windowMillis = TimeUnit.SECONDS.toMillis(windowSeconds);
        }

        public void recordEvent() {
            long now = System.currentTimeMillis();

            if (lastEventTime != -1) {
                long jitter = now - lastEventTime;
                measurements.addLast(new JitterMeasurement(now, jitter));
                cleanOldMeasurements(now);
            }

            lastEventTime = now;
        }

        private void cleanOldMeasurements(long now) {
            long windowStart = now - windowMillis;
            while (!measurements.isEmpty() && measurements.peekFirst().timestamp < windowStart) {
                measurements.removeFirst();
            }
        }

        public long getCountUnderThreshold(Duration threshold) {
            long thresholdMillis = threshold.toMillis();
            return measurements.stream()
                    .filter(m -> m.jitterMillis <= thresholdMillis)
                    .count();
        }

        public long getTotalCount() {
            return measurements.size();
        }

    }

    private JitterWithRatio createJitterMetrics(
            PrometheusMeterRegistry prometheusRegistry,
            String baseName,
            String description,
            Duration initialJitterThreshold,
            long windowSeconds) {

        JitterTracker tracker = new JitterTracker(windowSeconds);

        AtomicReference<Duration> jitterThreshold = new AtomicReference<>(initialJitterThreshold);

        // Gauge per il ratio (jitter sotto soglia / jitter totali)
        Gauge ratioGauge = Gauge.builder(baseName, () -> {
                    long underThreshold = tracker.getCountUnderThreshold(jitterThreshold.get());
                    long total = tracker.getTotalCount();

                    if (total == 0) {
                        return 0.0;
                    }

                    double ratio = (double) underThreshold / total;
                    return Math.min(1.0, Math.max(0.0, ratio));
                })
                .description(description + " - Ratio of jitter measurements under threshold")
                .register(prometheusRegistry);

        return new JitterWithRatio(tracker, ratioGauge, jitterThreshold);
    }

    public record PacketQualityMetrics(
            SlidingWindowCounter totalPacketsCounter,
            SlidingWindowCounter invalidPacketsCounter,
            Gauge validPacketsPercentageGauge,
            AtomicReference<Double> wantedPercentage,
            Gauge wantedPercentageGauge,
            Gauge qualityRatioGauge
    ) {}

    private PacketQualityMetrics createPacketQualityMetrics(
            PrometheusMeterRegistry prometheusRegistry,
            String baseName,
            String description,
            double initialWantedPercentage,
            long windowSeconds) {

        SlidingWindowCounter totalPackets = new SlidingWindowCounter(
                prometheusRegistry,
                baseName + "_total_packets",
                description + " - Total received packets",
                windowSeconds
        );

        SlidingWindowCounter invalidPackets = new SlidingWindowCounter(
                prometheusRegistry,
                baseName + "_invalid_packets",
                description + " - Invalid packets",
                windowSeconds
        );

        AtomicReference<Double> wantedPercentage = new AtomicReference<>(initialWantedPercentage);

        Gauge validPacketsPercentageGauge = Gauge.builder(
                        baseName + "_valid_packets_percentage",
                        () -> {
                            double total = totalPackets.getWindowTotalCount();
                            if (total == 0) {
                                return 0.0;
                            }

                            double invalid = invalidPackets.getWindowTotalCount();
                            double validPercentage = 1.0 - (invalid / total);

                            return Math.min(1.0, Math.max(0.0, validPercentage));
                        }
                )
                .description(description + " - Valid packets percentage (1 - invalid/total)")
                .register(prometheusRegistry);

        Gauge wantedPercentageGauge = Gauge.builder(
                        baseName + "_wanted_percentage",
                        wantedPercentage,
                        AtomicReference::get
                )
                .description(description + " - Wanted packets percentage")
                .register(prometheusRegistry);

        // Gauge per il ratio finale: valid_percentage / wanted_percentage
        Gauge qualityRatioGauge = Gauge.builder(
                        baseName + "_quality_ratio",
                        () -> {
                            double validPercentage = validPacketsPercentageGauge.value();
                            double wanted = wantedPercentage.get();

                            if (wanted == 0.0) {
                                return 0.0;
                            }
                            double ratio = validPercentage / wanted;
                            return Math.min(1.0, Math.max(0.0, ratio));
                        }
                )
                .description(description + " - Quality ratio (valid_percentage / wanted_percentage), max 1.0")
                .register(prometheusRegistry);

        return new PacketQualityMetrics(
                totalPackets,
                invalidPackets,
                validPacketsPercentageGauge,
                wantedPercentage,
                wantedPercentageGauge,
                qualityRatioGauge
        );
    }


    // Metrics exposed by Prometheus
    public final odteRatio reliabilityPhysicalToDigital = createMetrics(
            this.prometheusRegistry,
            droneId + "_updates_total_number_new_metric",
            "Total number of updates",
            droneId + "_expected_updates_rate",
            "Current expected updates rate per second",
            droneId + "_reliability_physical_to_digital",
            "Ratio between actual updates and expected rate",
            8
    );

    private final odteRatio availabilityDigitalToPhysical = createMetrics(
            this.prometheusRegistry,
            droneId + "_heartbeat_total",
            "Total number of heartbeat",
            droneId + "_expected_heartbeat_rate",
            "Current expected hb rate per second",
            droneId + "_availability_digital_to_physical",
            "Ratio between actual hb and expected rate",
            2.0);

    public final ReliabilityPhysicalToDigital reliabilityDigitalToPhysical = createReliabilityDigitalToPhysicalMetric(
            this.prometheusRegistry,
            droneId + "_action_total_count",
            "Total number of actions",
            droneId + "_expected_action_count",
            "Current expected actions rate per second",
            droneId + "_reliability_digital_to_physical",  // Nome del ratio
            "Ratio between actual actions and expected rate",
            30);


    /*public Timeliness physicalToDigitalTimeliness = createTimerWithRatio(
            "delta_time",
            "Delta time calculation: t1 + t2 - texec",
            "physical_to_digital_timeliness",
            "Ratio of delta times under 1 second (30s window)",
            0.5,
            Duration.ofSeconds(300),
            Duration.ofMillis(500),
            Duration.ofSeconds(300)
    );*/

    public final JitterWithRatio physicalToDigitalTimeliness = createJitterMetrics(
            prometheusRegistry,
            droneId + "_physical_to_digital_timeliness",
            "Jitter between pose updates",
            Duration.ofMillis(300),  // I want max n seconds between a message and another
            30
        );


    public final Timeliness digitalToPhysicalTimeliness = createTimerWithRatio(
            droneId + "_delta_actions_time",
            "Action delta time calculation: t1 + t2 - texec",
            droneId + "_digital_to_physical_timeliness",
            "Ratio of actions delta times under 1 second (30s window)",
            0.5,
            Duration.ofSeconds(300),
            Duration.ofMillis(100000),
            Duration.ofSeconds(300)
    );

    public final Gauge physicalToDigitalOdte = Gauge.builder(droneId + "_physical_to_digital_odte", () -> {
                double ratio1 = this.reliabilityPhysicalToDigital.ratioGauge().value();
                double physicalToDigitalTimelinessValue = this.physicalToDigitalTimeliness.ratioGauge().value();
                return ratio1 * physicalToDigitalTimelinessValue *
                        this.packetValidityMetric.validPacketsPercentageGauge.value() *
                        this.packetOutOfSeqMetric.validPacketsPercentageGauge.value();
            })
            .description("Product of updates and requests efficiency ratios")
            .register(prometheusRegistry);

    private final Gauge digitalToPhysicalOdte = Gauge.builder(droneId + "_digital_to_physical_odte", () -> {
                double ratio1 = this.reliabilityDigitalToPhysical.ratioGauge().value();
                double ratio2 = this.availabilityDigitalToPhysical.ratioGauge().value();
                double digitalToPhysicalTimelinessValue = this.digitalToPhysicalTimeliness.ratioGauge().value();
                return ratio1 * ratio2 * digitalToPhysicalTimelinessValue;
            })
            .description("Product of updates and requests efficiency ratios")
            .register(prometheusRegistry);

    public final PacketQualityMetrics packetValidityMetric = createPacketQualityMetrics(
            prometheusRegistry,
            droneId + "_pose_packets",
            "Packet validity ratio metrics",
            0.99,
            30
    );

    public final PacketQualityMetrics packetOutOfSeqMetric = createPacketQualityMetrics(
            prometheusRegistry,
            droneId + "_out_of_seq_packets",
            "Ratio between out-of-sequence packets and total packet count",
            0.99,
            30
    );


    /*public void setRequestedUpdatesValue(AtomicReference<Double> oldValue, double newValue) {
        oldValue.set(newValue);
    }*/

    public void incrementUpdateCounter(){
        this.reliabilityPhysicalToDigital.counter.increment();
    }

    public void incrementHeartbeatsCounter(){
        this.availabilityDigitalToPhysical.counter.increment();
    }

    public void incrementReuqestedActionsCounter(){
        this.reliabilityDigitalToPhysical.requestedCounter().increment();
    }

    public void incrementExecutedActionsCounter() {
        this.reliabilityDigitalToPhysical.executedCounter().increment();
    }

    public void incrementTotalPacketsCounter() {
        this.packetValidityMetric.totalPacketsCounter().increment();
    }

    public void incrementInvalidPacketsCounter() {
        this.packetValidityMetric.invalidPacketsCounter().increment();
    }

    public void incrementValidPacketsCounter() {
        this.packetOutOfSeqMetric.totalPacketsCounter().increment();
    }

    public void incrementOutOfSyncPacketsCounter() {
        this.packetOutOfSeqMetric.invalidPacketsCounter().increment();
    }

}
