package iot.drone.dt.adapter.physical.websocket.ros.rostopic;


import io.vertx.core.json.JsonObject;
import it.wldt.core.event.WldtEvent;

import java.util.List;
import java.util.function.Function;


@FunctionalInterface

public interface RosTopicSubscribeFunction extends Function<JsonObject, List<WldtEvent<?>>> {
}
