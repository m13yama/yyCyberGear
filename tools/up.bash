#!/bin/bash
# ============================================
# SocketCAN can0 interface initialization script
# ============================================

# Settings
IFACE="can0"
BITRATE=1000000     # 1Mbps
TX_QUEUE_LEN=1000   # TX queue length (default is ~10; increased)

echo "🔧 Setting up $IFACE (bitrate=${BITRATE}, txqueuelen=${TX_QUEUE_LEN})..."

# If the interface is already UP, bring it DOWN to reset
if ip link show "$IFACE" 2>/dev/null | grep -q "UP"; then
    echo "⚙️  $IFACE is currently UP. Bringing it down first..."
    sudo ip link set "$IFACE" down
    sleep 0.2
fi

# Configure interface bitrate
echo "⚙️  Configuring CAN bitrate..."
sudo ip link set "$IFACE" type can bitrate "$BITRATE"

# Increase transmit queue length
echo "⚙️  Setting txqueuelen to $TX_QUEUE_LEN..."
sudo ip link set "$IFACE" txqueuelen "$TX_QUEUE_LEN"

# Bring interface UP
echo "🚀 Bringing $IFACE UP..."
sudo ip link set "$IFACE" up

# Check interface status
echo "✅ Checking interface status..."
ip -details link show "$IFACE"

echo "✅ $IFACE is now UP and ready!"
