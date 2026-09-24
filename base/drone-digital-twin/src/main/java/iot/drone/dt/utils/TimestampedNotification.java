package iot.drone.dt.utils;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.NoArgsConstructor;
import lombok.ToString;

/*
 * This class stores the notification identifier and the timestamp
 * indicating when the notification was generated or received.
 */
@AllArgsConstructor
@NoArgsConstructor
@Data
@ToString
public class TimestampedNotification {

    public String id;
    public long timestamp;
}