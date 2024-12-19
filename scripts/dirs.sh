#!/bin/bash

weather_type="snow"

root_dir=$(dirname $(cd $(dirname $0);pwd))
exec_dir="$root_dir/bin"
data_dir="$root_dir/data/$weather_type"
og_dir="$data_dir/og"
sn_dir="$data_dir/sn"
sp_dir="$data_dir/sp"
rt_dir="$data_dir/rt"
lidsor_ep_dir="$rt_dir/lidsor/ep"
lidsor_en_dir="$rt_dir/lidsor/en"

check_dir(){
  if [ ! -d $1 ]
  then
    mkdir -p $1
  fi
}

check_dir $og_dir

# LIDSOR
check_dir $lidsor_ep_dir
check_dir $lidsor_en_dir

