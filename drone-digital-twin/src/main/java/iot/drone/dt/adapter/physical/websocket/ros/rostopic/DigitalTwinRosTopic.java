package iot.drone.dt.adapter.physical.websocket.ros.rostopic;

import io.github.twinklekhj.ros.op.RosSubscription;
import io.vertx.core.json.JsonObject;
import it.wldt.core.event.WldtEvent;

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

    public WldtEvent<?> applySubscribeFunction(JsonObject topicMessagePayload){
        return rosTopicSubscribeFunction.apply(topicMessagePayload);
    }

    public RosTopicSubscribeFunction getSubscribeFunction() {
        return rosTopicSubscribeFunction;
    }


    /*
    private final RosTopic topic;

    public RosTopic getTopic() {
        return topic;
    }

    private final RosTopicSubscribeFunction rosTopicSubscribeFunction;

    public DigitalTwinRosTopic(RosTopic topic, RosTopicSubscribeFunction rosTopicSubscribeFunction) {
        this.topic = topic;
        this.rosTopicSubscribeFunction = rosTopicSubscribeFunction;
    }

    public WldtEvent<?> applySubscribeFunction(JsonObject topicMessagePayload){
        return rosTopicSubscribeFunction.apply(topicMessagePayload);
    }

    public RosTopicSubscribeFunction getSubscribeFunction() {
        return rosTopicSubscribeFunction;
    }*/

}
