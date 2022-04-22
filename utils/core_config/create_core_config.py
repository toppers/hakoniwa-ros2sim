#!/usr/bin/python
# -*- coding: utf-8 -*-
import json
import sys
import glob
import re
from collections import OrderedDict

if len(sys.argv) != 4:
	print "Usage: " + sys.argv[0] + " <ros_topic_path> <ipaddr> <out_dir>"
	sys.exit()


ros_topic_path=sys.argv[1]
ipaddr=sys.argv[2]
out_dir=sys.argv[3]

out_data = OrderedDict()

out_data['core_ipaddr'] = ipaddr
out_data['core_portno'] = 50051
out_data['asset_timeout'] = 3
out_data['inside_assets_path'] = './inside_assets.json'
out_data['outside_assets_path'] = './outside_assets.json'
out_data['pdu_writers_path'] = './pdu_writers_config.json'
out_data['pdu_readers_path'] = './pdu_readers_config.json'
out_data['pdu_configs_path'] = './pdu_configs.json'
out_data['ros_topics_path'] = ros_topic_path
out_data['ros_topic_method_path'] = './ros_topic_method.json'
out_data['udp_methods_path'] = './udp_methods.json'
out_data['mmap_methods_path'] = './mmap_methods.json'
out_data['reader_connectors_path'] = './reader_connectors.json'
out_data['writer_connectors_path'] = './writer_connectors.json'
out_data['pdu_channel_connectors_path'] = './pdu_channel_connectors.json'
out_data['param_world_config_path'] = './param_world.json'

with open(out_dir + '/core_config.json', mode='wt') as out_file:
  json.dump(out_data, out_file, ensure_ascii=False, indent=2)

