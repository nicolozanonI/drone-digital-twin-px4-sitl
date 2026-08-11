package iot.drone.dt.adapter.physical.websocket.ros;



import iot.drone.dt.adapter.physical.websocket.ros.exception.WebSocketRosPhysicalAdapterException;
import iot.drone.dt.adapter.physical.websocket.ros.rostopic.DigitalTwinRosTopic;
import it.wldt.adapter.physical.ConfigurablePhysicalAdapter;
import it.wldt.adapter.physical.event.PhysicalAssetActionWldtEvent;
import it.wldt.adapter.physical.event.PhysicalAssetEventWldtEvent;
import it.wldt.adapter.physical.event.PhysicalAssetPropertyWldtEvent;
import it.wldt.core.event.WldtEvent;
import it.wldt.exception.EventBusException;
import it.wldt.exception.PhysicalAdapterException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.lang.reflect.InvocationTargetException;
import java.util.List;
import java.util.Random;

public class WebSocketRosPhysicalAdapter extends ConfigurablePhysicalAdapter<WebSocketRosPhysicalAdapterConfiguration> {

    private static final Logger logger = LoggerFactory.getLogger(WebSocketRosPhysicalAdapter.class);

    @Override
    public void onIncomingPhysicalAction(PhysicalAssetActionWldtEvent<?> physicalActionEvent) {
        //Gson gson = new Gson();
        logger.info("WebSocket Physical Adapter received action event: {}", physicalActionEvent);
        getConfiguration()
                .getOutgoingMessageByActionKey(physicalActionEvent.getActionKey())
                .ifPresent(k -> {
                    try {
                        logger.info("{}", physicalActionEvent.getBody());
                        getConfiguration().applyPublishMethodByActionKey(physicalActionEvent.getActionKey(),
                                getConfiguration().getSendFunctions(), physicalActionEvent.getBody().toString().getBytes(),
                                getConfiguration().getWebSocketAdapterClient().getBridge());
                    } catch (InvocationTargetException | IllegalAccessException e) {
                        throw new RuntimeException(e);
                    }
                });
    }

    public void onPhysicalPropertyUpdate(PhysicalAssetPropertyWldtEvent<?> physicalAssetPropertyWldtEvent) {

    }

    private void subscribeClientToDigitalTwinIncomingTopic(String s, DigitalTwinRosTopic digitalTwinRosTopic) {
        getConfiguration().getWebSocketAdapterClient().getBridge().subscribe(
                digitalTwinRosTopic.getRosSubscription(),
                message -> {
                    List<WldtEvent<?>> wldtEvents = digitalTwinRosTopic.applySubscribeFunction(message.body());
                    if (wldtEvents == null) {
                        logger.error("applySubscribeFunction returned null for topic {}", s);
                        return;
                    }

                    wldtEvents.forEach(event -> {
                        logger.info("MESSAGE ARRIVED ON {} TOPIC", s);
                        try {
                            if (event instanceof PhysicalAssetPropertyWldtEvent) {
                                publishPhysicalAssetPropertyWldtEvent((PhysicalAssetPropertyWldtEvent<?>) event);
                            } else if (event instanceof PhysicalAssetEventWldtEvent) {
                                publishPhysicalAssetEventWldtEvent((PhysicalAssetEventWldtEvent<?>) event);
                            }
                        } catch (EventBusException ex) {
                            ex.printStackTrace();
                        }
                    });
                }
        );
    }

    @Override
    public void onAdapterStart() {
        try {
            int maxWait = 5000;
            long waitTime = new Random().nextInt(maxWait);
            Thread.sleep(waitTime);
            this.getConfiguration().getWebSocketAdapterClient().connect();
            //Thread.sleep(10000);
            getConfiguration().getIncomingMessages().forEach(this::subscribeClientToDigitalTwinIncomingTopic);
            notifyPhysicalAdapterBound(getConfiguration().getPhysicalAssetDescription());
        } catch (PhysicalAdapterException | EventBusException e) {
            e.printStackTrace();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }


    @Override
    public void onAdapterStop() {
        this.getConfiguration().getWebSocketAdapterClient().disconnect();
    }

    public WebSocketRosPhysicalAdapter(String id, WebSocketRosPhysicalAdapterConfiguration configuration) throws WebSocketRosPhysicalAdapterException {
        super(id, configuration);
    }


}
