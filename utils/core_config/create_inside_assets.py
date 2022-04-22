#!/usr/bin/python
# -*- coding: utf-8 -*-
import json
import sys
import glob
import re
from collections import OrderedDict

import hakoniwa_utils

if len(sys.argv) != 3:
	print "Usage: " + sys.argv[0] + " <ros_tpics> <out_dir>"
	sys.exit()

in_file=sys.argv[1]
out_dir=sys.argv[2]
out_filename='inside_assets.json'

file = open(in_file)
ros_topics = json.load(file)

robo_list = hakoniwa_utils.get_robolist(ros_topics)

container = list()
for robo in robo_list:
	e_list = hakoniwa_utils.get_entry(ros_topics, 'robot_name', robo)
	entry = OrderedDict()
	entry['name'] = robo
	entry['pdu_writer_names'] = list()
	entry['pdu_reader_names'] = list()
	for e_list_entry in e_list:
		if e_list_entry['sub']:
			entry['pdu_reader_names'].append(hakoniwa_utils.get_pdu_name(e_list_entry))
		else:
			entry['pdu_writer_names'].append(hakoniwa_utils.get_pdu_name(e_list_entry))
	container.append(entry)

with open(out_dir + '/' + out_filename, mode='wt') as out_file:
  json.dump(container, out_file, ensure_ascii=False, indent=2)

