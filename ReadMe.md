# Masik Part
본 레포지토리는 GPS에 NTRIP를 연결하는 도구와 그 방법을 설명합니다.
아래 링크와 밀접한 관계가 있습니다.
[aktmdtkd/ublox_F9R](https://github.com/aktmdtkd/ublox_F9R)

여기서 본래 config 파일이 없는데, 추가해서 그 안에 'ntrip.yaml'파일을 만들어야 합니다.

```
caster: ?
port: ?
mountpoint: ?
username: ?
password: ?
serial_device: /dev/ttyACM?
```

이런식으로 적혀있을 건데, 여기서 serial_device가 중요합니다.
GPS를 연결하지 않았다가 연결하면 몇개의 ttyACM이 생기는지 보셔야 합니다.
그렇게 생기는 것이 2개라면, 하나는 위의 ublox_F9R에서 사용하시고, 다른 하나를 ntrip에 연결해야 합니다.

**둘을 같은 포트로 사용하시면 충돌납니다.**

---

# NTRIP Client

ROS 2 package for publishing RTCM correction data from an NTRIP server.

### Publishers

|Topic | Type | Description  |
|---------|---------|---------|
| `/rtcm` | [`rtcm_msgs/msg/Message`](https://github.com/tilk/rtcm_msgs/blob/master/msg/Message.msg) | RTCM Correction data |


### Parameters
|Parameter | Type  |   Values  | Runtime R/W | Description  |
|---------|---------|---------|---------|---------|
| `use_https` | bool |  `true/false` | `read-only` | HTTPS will be used if set true. | 
| `host` | string |   | `read-only` | NTRIP sever hostname |
| `port` | string | `2101/2102`| `read-only`  | NTRIP server port |
| `mountpoint` | string |  |  `read-only`|  Mountpoint to connect |
| `username` | string |  | `read-only` | Login credentials for the NTRIP subscription  |
| `password` | string |  | `read-only` | Login credentials for the NTRIP subscription  |


## RTK Correction Services

RTK (Real-Time Kinematic) correction service is a technique used to enhance the precision of position data derived from satellite-based positioning systems such as GPS, GLONASS, Galileo, and BeiDou. RTK achieves centimeter-level accuracy by using corrections transmitted from a network of ground-based reference stations to a receiver in real time. 

Here is a free RTK correction service that you can use with the NTRIP client.

### RTK2go

RTK2go is a community NTRIP Caster that provides RTK correction service for free. To use it, select a mountpoint near your location. You can find the list of available mountpoints [here](http://monitor.use-snip.com/map). Click on **`view-all`** to view mountpoints on the map.

Once you have the mountpoint you would like to connect to, add that to the config file as shown below.


```yml
ntrip_client:
  ros__parameters:
    use_https: false
    host: rtk2go.com
    port: 2101
    mountpoint: Prittlbach  # Add your mointpoint here
    username: "noname"
    password: "password"
```
