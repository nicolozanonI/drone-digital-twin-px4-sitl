package iot.drone.dt.adapter.physical.websocket.ros;

import io.github.twinklekhj.ros.core.ConnProps;
import io.github.twinklekhj.ros.core.RosBridge;
import io.vertx.core.Vertx;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class RosBridgeClient {

    private final String host;
    private final Integer port;
    private static final Logger logger = LoggerFactory.getLogger(RosBridgeClient.class);

    private final RosBridge bridge;

    public RosBridgeClient(String host, Integer port) {
        this.host = host;
        this.port = port;
        Vertx vertx = Vertx.vertx();
        ConnProps address = new ConnProps();
        address.setHost(host);
        address.setPort(port);
        this.bridge = new RosBridge(vertx, address);
    }

    public void connect() {
        this.bridge.start();
        logger.info("RosBridge Client is Connected !");
    }
    
    public void disconnect() {
        this.bridge.close();
    }

    public RosBridge getBridge() {
        return bridge;
    }


}
