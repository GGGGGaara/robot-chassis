#!/usr/bin/env bash
# WSL-only DDS loopback fix for isolated demo domain 74; never touches CAN.
set -eo pipefail
if ! grep -qi microsoft /proc/sys/kernel/osrelease; then exit 0; fi
if ip -4 route get 127.0.0.1 ipproto udp dport 25910 | grep -q ' dev lo '; then exit 0; fi
if [[ $EUID != 0 ]]; then exec sudo bash "${BASH_SOURCE[0]}"; fi
ip link set lo multicast on
ip -4 route replace 239.255.0.1/32 dev lo
ip -4 rule add priority 0 to 127.0.0.1 ipproto udp dport 25900-26149 lookup local
ip -4 route get 127.0.0.1 ipproto udp dport 25910 | grep -q ' dev lo '
