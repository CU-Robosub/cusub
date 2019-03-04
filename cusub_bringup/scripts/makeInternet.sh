#!/bin/bash

# gives everything connected to robo local network access to internet
# through computer commands are run
# computer used to run commands looses ability to http router, but can still ssh/ping everything
# (working on fixing that)
# to look at script that does dhcp & can make rules persistent:
# https://gist.github.com/Will-Shanks/bc2d4cedcdcfc17880a06c2eb0cd0403

# set local ip to routers gateway (on router webpage)
# set routers dns to 8.8.8.8 and/or 8.8.4.4 (on router webpage)

echo "Don't forget to set router's default gateway & DNS"

echo "robo interface: (ex: eth0)"
read LOCAL_INTERFACE

echo "internet interface:"
read EXTERNAL_INTERFACE

echo "router's ip:"
read $ROBO_ROUTER_IP

#enable packet forwarding
sysctl net.ipv4.ip_forward=1

#nat magic
iptables -t nat -A POSTROUTING -o $EXTERNAL_INTERFACE -j MASQUERADE
iptables -A FORWARD -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT
iptables -A FORWARD -i $LOCAL_INTERFACE -o $EXTERNAL_INTERFACE -j ACCEPT

# so comp doesn't try and use robo router as default gateway (making a loop)
ip r del default via $ROBO_ROUTER_IP dev $LOCAL_INTERFACE
