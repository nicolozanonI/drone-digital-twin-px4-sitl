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


    /**/
    /*
    @Override
    public void onIncomingPhysicalAction(PhysicalAssetActionWldtEvent<?> physicalActionEvent) {
        logger.info("WebSocket Physical Adapter received action event: {}", physicalActionEvent);

        getConfiguration()
                .getOutgoingMessageByActionKey(physicalActionEvent.getActionKey())
                .ifPresent(k -> {
                    try {
                        logger.info("{}", physicalActionEvent.getBody());

                        // Ottieni l'oggetto (Method o Function) dalla mappa
                        Object sendFunction = getConfiguration().getSendFunctionByActionKey(
                                physicalActionEvent.getActionKey()
                        );

                        // Esegui il metodo appropriato in base al tipo e cattura il response
                        Object response = executeSendFunction(
                                sendFunction,
                                physicalActionEvent.getBody().toString().getBytes(),
                                getConfiguration().getWebSocketAdapterClient().getBridge()
                        );

                        // Applica un metodo al response
                        if (response != null) {
                            processResponse(response);
                        }

                    } catch (Exception e) {
                        logger.error("Error executing send function", e);
                        throw new RuntimeException(e);
                    }
                });
    }

    // Metodo helper per eseguire sia Method che Function
    private Object executeSendFunction(Object sendFunction, byte[] data, Object bridge)
            throws InvocationTargetException, IllegalAccessException {

        if (sendFunction instanceof Function) {
            // Se è una Function, assumiamo che accetti parametri e ritorni qualcosa
            // Adatta in base alla tua signature specifica
            @SuppressWarnings("unchecked")
            Function<Object[], Object> function = (Function<Object[], Object>) sendFunction;
            return function.apply(new Object[]{data, bridge});

        } else if (sendFunction instanceof Method) {
            // Se è un Method (Reflection), invocalo e ritorna il risultato
            Method method = (Method) sendFunction;
            return method.invoke(null, data, bridge);

        } else {
            throw new IllegalArgumentException(
                    "Unsupported send function type: " + sendFunction.getClass()
            );
        }
    }

    // Metodo per processare il response
    private void processResponse(Object response) {
        logger.info("Processing response: {}", response);

        // Esempio: converti il response in String e fai qualcosa
        String responseString = response.toString();

        // Applica logica personalizzata
        if (responseString.contains("success")) {
            logger.info("Operation completed successfully");
        } else {
            logger.warn("Operation returned: {}", responseString);
        }

        // Oppure chiama un altro metodo sul response
        // publishResponseEvent(response);
    }*/
    /**/


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
