#!/bin/bash
# Create ~/.ros/fastdds.xml for Isaac Sim + ROS2 (Fast DDS).
# Run once before using Isaac Sim with ROS2 bridge.

mkdir -p "$HOME/.ros"
cat > "$HOME/.ros/fastdds.xml" << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>UdpTransport</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>
</profiles>
EOF
echo "Created $HOME/.ros/fastdds.xml"
