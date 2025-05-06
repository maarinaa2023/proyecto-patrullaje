#!/bin/bash
# FastDDS Discovery Server startup script

# Default values
SERVER_ID=${1:-69}
SERVER_IP=${2:-10.0.0.1}
SERVER_PORT=${3:-11811}

#echo "Starting FastDDS Discovery Server with:"
#echo "  - Server ID: $SERVER_ID"
#echo "  - IP Address: $SERVER_IP"
#echo "  - Port: $SERVER_PORT"

# Start the discovery server
fastdds discovery -i $SERVER_ID -l $SERVER_IP -p $SERVER_PORT

