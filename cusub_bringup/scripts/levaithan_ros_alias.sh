#!/bin/bash
if [[ -z "$1" ]]; then
  if [[ -z "${SUB_IP}" ]]; then
    SUB_IP='http://10.0.0.10:11311'
  fi
else
  SUB_IP="$1"
fi
#echo $SUB_IP
cd ..
(echo -n "alias leviathan='source " ;echo -n $PWD; echo -n "/devel/setup.bash";
echo "; ROS_MASTER_URI=$SUB_IP'";) >> ~/.bash_aliases

source ~/.bashrc
