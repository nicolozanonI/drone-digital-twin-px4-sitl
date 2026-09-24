package iot.entanglement.drone.dt.ros_physical_adapter.rostopic;


import io.vertx.core.json.JsonObject;
import it.wldt.core.event.WldtEvent;

import java.util.List;
import java.util.function.Function;


@FunctionalInterface

public interface RosTopicSubscribeFunction extends Function<JsonObject, List<WldtEvent<?>>> {
}
