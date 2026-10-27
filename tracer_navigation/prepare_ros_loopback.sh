#!/usr/bin/env bash
# Domain 42: ports 17900..18149; domain 43 tests: 18150..18399.
# WSL mirrored networking redirects 127.0.0.1 through table 127 by default.
set -eo pipefail
if ip -4 route get 127.0.0.1 ipproto udp dport 17910 | grep -q ' dev lo ' &&
   ip -o link show lo | grep -q 'MULTICAST' &&
   ip -4 route get 239.255.0.1 | grep -q ' dev lo '; then
  echo 'ROS UDP loopback route ready.'; exit 0
fi
if [[ $EUID != 0 ]]; then exec sudo bash "${BASH_SOURCE[0]}"; fi
ip link set lo multicast on
ip -4 route replace 239.255.0.1/32 dev lo
added_rule=false
if ! ip -4 route get 127.0.0.1 ipproto udp dport 17910 | grep -q ' dev lo '; then
  ip -4 rule add priority 0 to 127.0.0.1 ipproto udp dport 17900-18399 lookup local
  added_rule=true
fi
if ! ip -4 route get 127.0.0.1 ipproto udp dport 17910 | grep -q ' dev lo '; then
  if $added_rule; then ip -4 rule del priority 0 to 127.0.0.1 ipproto udp dport 17900-18399 lookup local; fi
  echo 'ROS UDP loopback route verification failed.'; exit 1
fi
echo 'ROS UDP ports 17900..18399 now use Linux loopback.'
