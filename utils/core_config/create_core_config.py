#!/usr/bin/python
# -*- coding: utf-8 -*-
import json
import sys
import glob
import re
from collections import OrderedDict

if len(sys.argv) != 7:
	print "Usage: " + sys.argv[0] + " <ros_topic_path> <ros_topic_file> <ipaddr1> <ipaddr2> <is_asset> <out_dir>"
	sys.exit()


ros_topic_path=sys.argv[1]
in_file=sys.argv[2]
ipaddr1=sys.argv[3]
ipaddr2=sys.argv[4]
is_asset=sys.argv[5]
out_dir=sys.argv[6]

file = open(in_file)
ros_topics = json.load(file)

out_data = OrderedDict()

if is_asset == "YES_RPC_MODE":
	out_data['core_ipaddr'] = ipaddr2
	out_data['asset_ipaddr'] = ipaddr1
	out_data['cpp_mode'] = 'asset_rpc'
	out_data['cpp_asset_name'] = 'UnityAssetRpc'
	out_data['pdu_udp_portno_asset'] = 54003
	out_data['core_portno'] = 50051
elif is_asset == "YES_DIRECT_MODE":
	out_data['cpp_mode'] = 'asset'
	out_data['cpp_asset_name'] = 'UnityAsset'
else:
	out_data['core_ipaddr'] = ipaddr1
	out_data['core_portno'] = 50051

out_data['asset_timeout'] = 3
out_data['pdu_bin_offset_package_dir'] = './offset'

out_data['inside_assets_path'] = './inside_assets.json'

out_data['outside_assets_path'] = None

out_data['pdu_writers_path'] = './pdu_writers_config.json'
out_data['pdu_readers_path'] = './pdu_readers_config.json'
out_data['pdu_configs_path'] = './pdu_configs.json'
out_data['ros_topics_path'] = ros_topic_path
if ros_topics['ros_robot_num'] == 0:
	out_data['ros_topic_method_path'] = None
else:
	out_data['ros_topic_method_path'] = './ros_topic_method.json'

out_data['udp_methods_path'] = './udp_methods.json'
out_data['rpc_methods_path'] = './rpc_methods.json'
out_data['shm_methods_path'] = './shm_methods.json'
out_data['mmap_methods_path'] = './mmap_methods.json'
out_data['reader_connectors_path'] = './reader_connectors.json'
out_data['writer_connectors_path'] = './writer_connectors.json'
out_data['pdu_channel_connectors_path'] = './pdu_channel_connectors.json'
out_data['param_world_config_path'] = './param_world.json'

with open(out_dir + '/core_config.json', mode='wt') as out_file:
  json.dump(out_data, out_file, ensure_ascii=False, indent=2)

