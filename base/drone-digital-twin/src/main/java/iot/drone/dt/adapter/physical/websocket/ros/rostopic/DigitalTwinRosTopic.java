package iot.drone.dt.adapter.physical.websocket.ros.rostopic;

import io.github.twinklekhj.ros.op.RosSubscription;
import io.vertx.core.json.JsonObject;
import it.wldt.core.event.WldtEvent;

import java.util.List;

public class DigitalTwinRosTopic {


    private final RosSubscription rosSubscription;

    public RosSubscription getRosSubscription() {
        return rosSubscription;
    }

    private final RosTopicSubscribeFunction rosTopicSubscribeFunction;

    public DigitalTwinRosTopic(RosSubscription rosSubscription, RosTopicSubscribeFunction rosTopicSubscribeFunction) {
        this.rosSubscription = rosSubscription;
        this.rosTopicSubscribeFunction = rosTopicSubscribeFunction;
    }

    public List<WldtEvent<?>> applySubscribeFunction(JsonObject topicMessagePayload){
        return rosTopicSubscribeFunction.apply(topicMessagePayload);
    }

    public RosTopicSubscribeFunction getSubscribeFunction() {
        return rosTopicSubscribeFunction;
    }

}
