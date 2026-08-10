package iot.drone.dt.adapter.physical.websocket.ros;

import io.github.twinklekhj.ros.core.RosBridge;
import iot.drone.dt.adapter.physical.websocket.ros.rostopic.DigitalTwinRosTopic;
import it.wldt.adapter.physical.PhysicalAssetAction;
import it.wldt.adapter.physical.PhysicalAssetDescription;
import it.wldt.adapter.physical.PhysicalAssetEvent;
import it.wldt.adapter.physical.PhysicalAssetProperty;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.*;



public class WebSocketRosPhysicalAdapterConfiguration {

    private static final Logger logger = LoggerFactory.getLogger(WebSocketRosPhysicalAdapterConfiguration.class);

    private final String id;
    private String host;
    private Integer port;

    private Object sendFunctions;
    private final RosBridgeClient webSocketAdapterClient;

    private final List<PhysicalAssetEvent> events = new ArrayList<>();

    private final List<PhysicalAssetProperty<?>> properties = new ArrayList<>();

    private final List<PhysicalAssetAction> actions = new ArrayList<>();
    private final Map<String, DigitalTwinRosTopic> incomingMessages = new HashMap<>();

    private final Map<String, Method> outgoingMessages = new HashMap<>();


    private PhysicalAssetDescription physicalAssetDescription;

    public Map<String, Method> getOutgoingMessages() {
        return outgoingMessages;
    }

    public Map<String, DigitalTwinRosTopic> getIncomingMessages() {
        return incomingMessages;
    }

    public WebSocketRosPhysicalAdapterConfiguration(String id, String host, Integer port,
                                                    RosBridgeClient webSocketAdapterClient) {
        this.id = id;
        this.webSocketAdapterClient = webSocketAdapterClient;
    }

    public WebSocketRosPhysicalAdapterConfiguration(String id,
                                                    RosBridgeClient webSocketAdapterClient, Object sendFunctions) {
        this.id = id;
        this.webSocketAdapterClient = webSocketAdapterClient;
        this.sendFunctions = sendFunctions;
    }

    public Object getSendFunctions() {
        return sendFunctions;
    }

    public String getId() {
        return id;
    }

    public String getHost() {
        return host;
    }

    public Integer getPort() {
        return port;
    }

    public RosBridgeClient getWebSocketAdapterClient() {
        return webSocketAdapterClient;
    }

    public List<PhysicalAssetProperty<?>> getProperties() {
        return properties;
    }

    public List<PhysicalAssetEvent> getEvents() {
        return events;
    }

    public List<PhysicalAssetAction> getActions() {
        return actions;
    }

    public PhysicalAssetDescription getPhysicalAssetDescription() {
        return physicalAssetDescription;
    }

    public void setPhysicalAssetDescription(List<PhysicalAssetAction> actions,
                                            List<PhysicalAssetProperty<?>> properties,
                                            List<PhysicalAssetEvent> events) {
        this.physicalAssetDescription = new PhysicalAssetDescription(actions, properties, events);
    }

    public WebSocketRosPhysicalAdapterConfiguration addPhysicalAssetActionMethod(String actionKey, String type, String contentType, Method sendMethod){
        this.outgoingMessages.put(actionKey, sendMethod);
        return addPhysicalAssetAction(actionKey, type, contentType);
    }

    /* Modifiche per gestione Request/Response

    public WebSocketRosPhysicalAdapterConfiguration addPhysicalAssetActionMethod(String actionKey, String type, String contentType, Function<Void, String> sendFunctions){
        this.outgoingMessages.put(actionKey, sendFunctions);
        return addPhysicalAssetAction(actionKey, type, contentType);
    }
*/
    public <T> WebSocketRosPhysicalAdapterConfiguration addPhysicalAssetPropertyTopic(String propertyKey, T initialValue, DigitalTwinRosTopic digitalTwinRosTopic){
        this.incomingMessages.put(propertyKey, digitalTwinRosTopic);
        return addPhysicalAssetProperty(propertyKey, initialValue);
    }

    public <T> WebSocketRosPhysicalAdapterConfiguration addMultiplePhysicalAssetPropertyTopics(Map<String, T> keyValueMap, DigitalTwinRosTopic digitalTwinRosTopic) {
        for (String propertyKey : keyValueMap.keySet()) {
            this.incomingMessages.put(propertyKey, digitalTwinRosTopic);
        }
        return addMultiplePhysicalAssetProperties(keyValueMap);
    }

    public <T> WebSocketRosPhysicalAdapterConfiguration addPhysicalAssetEventTopic(String eventKey, String type, DigitalTwinRosTopic digitalTwinRosTopic){
        this.incomingMessages.put(eventKey, digitalTwinRosTopic);
        return addPhysicalAssetEvent(eventKey, type);
    }


    public Optional<Method> getOutgoingMessageByActionKey(String key){
        return outgoingMessages.containsKey(key) ? Optional.of(outgoingMessages.get(key)) : Optional.empty();
    }


    public void applyPublishMethodByActionKey(String key, Object object, byte[] data, RosBridge rosBridge) throws InvocationTargetException, IllegalAccessException {
        if (outgoingMessages.containsKey(key)) {
            outgoingMessages.get(key).invoke(object, data, rosBridge);
        }
    }


    private WebSocketRosPhysicalAdapterConfiguration addPhysicalAssetAction(String key, String type, String contentType){
        this.actions.add(new PhysicalAssetAction(key, type, contentType));
        return this;
    }

    private <T> WebSocketRosPhysicalAdapterConfiguration addMultiplePhysicalAssetProperties(Map<String, T> keyValuePairs) {
        for (Map.Entry<String, T> entry : keyValuePairs.entrySet()) {
            this.properties.add(new PhysicalAssetProperty<>(entry.getKey(), entry.getValue()));
        }
        return this;
    }

    private <T> WebSocketRosPhysicalAdapterConfiguration addPhysicalAssetProperty(String key, T initialValue){
        this.properties.add(new PhysicalAssetProperty<>(key, initialValue));
        return this;
    }

    public WebSocketRosPhysicalAdapterConfiguration addPhysicalAssetEvent(String key, String type){
        this.events.add(new PhysicalAssetEvent(key, type));
        return this;
    }

    public WebSocketRosPhysicalAdapterConfiguration build() {
        this.setPhysicalAssetDescription(actions, properties, events);
        return this;
    }

}
