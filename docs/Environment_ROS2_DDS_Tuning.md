# ROS 2 Performance Tuning: The "Pi Zero" Profile
**"512MB is plenty... if you're careful."**

By default, ROS 2 is configured for powerful laptops. On a Raspberry Pi Zero 2 W, the default "Discovery" process will cause a 100% CPU spike every time a new node starts.

---

## **1. Use a Shared Memory Transport**
If your nodes are all running on the same Pi, they shouldn't use the Network Stack (UDP) to talk to each other.
*   **The Fix:** Configure **Iceoryx** or a Shared Memory transport in your `USER_ATTRIBUTES.xml`. This reduces latency from milliseconds to microseconds.

---

## **2. Disable Multicast Discovery**
The Pi Zero's Wi-Fi chip is weak. Constant "Who is there?" packets will cause Jitter.
*   **The Fix:** Use a **Discovery Server**. One node (the laptop) acts as the "Phonebook," and the Pi only talks to that server.

---

## **3. XML Configuration (`fastdds_profile.xml`)**
Place this file on your Pi and export it: `export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_profile.xml`

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles>
    <participant profile_name="pi_zero_profile">
        <rtps>
            <!-- Reduce memory footprint -->
            <allocation>
                <remote_locators>
                    <max_unicast_locators>2</max_unicast_locators>
                </remote_locators>
            </allocation>
            <!-- Use a static discovery server on the Laptop IP -->
            <builtin>
                <discovery_config>
                    <discoveryProtocol>CLIENT</discoveryProtocol>
                    <discoveryServersList>
                        <remoteServer guidPrefix="44.53.01.5f.45.50.52.4f.53.49.4d.41">
                            <metatrafficUnicastLocatorList>
                                <locator><udpv4><address>192.168.1.XX</address><port>11811</port></udpv4></locator>
                            </metatrafficUnicastLocatorList>
                        </remoteServer>
                    </discoveryServersList>
                </discovery_config>
            </builtin>
        </rtps>
    </participant>
</profiles>
```

---

## **4. QoS (Quality of Service)**
*   **Sensors:** Use `Best Effort` (don't retry lost packets).
*   **Commands:** Use `Reliable` (ensure motor commands arrive).
*   **Depth:** Keep your queue depth at **1**. We only ever care about the *latest* data.
