# Network Configuration: LIO-SAM with SICK MultiScan & WitMotion IMU

## Current Network Configuration

### SICK MultiScan LiDAR
- **IP Address**: 192.168.31.174
- **Port**: 2122 (TCP)
- **Protocol**: TCP/IP for control, UDP for data
- **UDP Receiver**: 192.168.31.146 (destination for point cloud data)

### Host Computer
- **Ethernet IP**: 192.168.31.146
- **Subnet Mask**: 255.255.255.0
- **Interface**: Configured via netplan

## Network Topology

```
+-------------------+                                   +------------------+
|                   |                                   |                  |
| SICK MultiScan    |                                   | Host Computer    |
| 192.168.31.174    |<---------------------------+----->| 192.168.31.146   |
|                   |  Ethernet (TCP/IP & UDP)   |      |                  |
+-------------------+                            |      +------------------+
                                                 |                |
                                                 |                | USB
                                                 |                v
                                                 |      +------------------+
                                                 |      | WitMotion IMU    |
                                                 |      | /dev/ttyIMU      |
                                                 |      |                  |
                                                 |      +------------------+
```

## Network Setup Instructions

### 1. Configure Host Computer's Network Interface

Create or edit netplan configuration:

```bash
# ~/
sudo nano /etc/netplan/01-network-manager-all.yaml
```

Add the following configuration:

```yaml
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    eth0:  # Change to match your interface name (check with 'ip a')
      dhcp4: no
      addresses: [192.168.31.146/24]
      optional: true
```

Apply the configuration:

```bash
# ~/
sudo netplan apply
```

### 2. Verify Network Connection

Ping the LiDAR to verify connectivity:

```bash
# ~/
ping 192.168.31.174
```

### 3. LiDAR Configuration

The LiDAR should be configured with:
- IP address: 192.168.31.174
- Network mask: 255.255.255.0
- Default gateway: (not required for direct connection)

If changes to LiDAR IP are needed, use the SICK SOPAS Engineering Tool.

## Troubleshooting

### Common Issues

1. **Cannot connect to LiDAR**
   - Verify physical connection and cables
   - Check IP address configuration
   - Verify no firewall rules blocking the connection

2. **Data packets not received**
   - Verify UDP receiver IP is correct
   - Check for network congestion or packet loss
   - Verify no firewall blocking UDP traffic

3. **Intermittent connection**
   - Check cable quality and connection
   - Verify power supply to LiDAR is stable
   - Monitor network traffic for interference

### Network Testing Commands

```bash
# Check connectivity
ping 192.168.31.174

# Check network interface
ip addr show

# Check network traffic
sudo tcpdump -i eth0 host 192.168.31.174

# Check open ports
sudo netstat -tuln | grep 2122
``` 