#!/usr/bin/python
# -*- coding: utf-8 -*-
import json
import sys
import glob
import re
from collections import OrderedDict

import hakoniwa_utils

if len(sys.argv) != 4:
	print "Usage: " + sys.argv[0] + " <custom> <core_ipaddr> <out_dir>"
	sys.exit()

in_file=sys.argv[1]
core_ipaddr=sys.argv[2]
out_dir=sys.argv[3]
out_filename='udp_methods.json'

file = open(in_file)
custom = json.load(file)


container = list()
for robo in custom['proxies']:

	#writer
	entry = OrderedDict()
	entry['method_name'] = robo['robot_name'] + 'Proxy_UdpWriter'
	if robo['proxy_sync']['ipaddr'] != None:
		entry['ipaddr'] = robo['proxy_sync']['ipaddr']
	else:
		entry['ipaddr'] = '127.0.0.1'
	entry['portno'] = robo['proxy_sync']['tx_udp_portno']
	entry['iosize'] = 1024
	entry['is_read'] = False
	container.append(entry)

	#reader
	entry = OrderedDict()
	entry['method_name'] = robo['robot_name'] + 'Proxy_UdpReader'
	entry['ipaddr'] = core_ipaddr
	entry['portno'] = robo['proxy_sync']['rx_udp_portno']
	entry['iosize'] = 1024
	entry['is_read'] = True
	container.append(entry)

with open(out_dir + '/' + out_filename, mode='wt') as out_file:
  json.dump(container, out_file, ensure_ascii=False, indent=2)

