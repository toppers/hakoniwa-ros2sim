#!/usr/bin/python
# -*- coding: utf-8 -*-
import json
import sys
import glob
import re
import os
from collections import OrderedDict

import hakoniwa_utils

if len(sys.argv) != 6:
	print "Usage: " + sys.argv[0] + " <ros_tpics> <out_dir> <pkg_name> {r|w} <custom>"
	sys.exit()

in_file=sys.argv[1]
out_dir=sys.argv[2]
pkg_name=sys.argv[3]
rw_type=sys.argv[4]

custom=None
custom_file=sys.argv[5]
if os.path.exists(custom_file):
	tmp_file = open(custom_file)
	custom = json.load(tmp_file)

if rw_type == 'r':
	out_filename='pdu_readers_config.json'
else:
	out_filename='pdu_writers_config.json'

file = open(in_file)
ros_topics = json.load(file)


container = list()
for e in ros_topics['fields']:
	if hakoniwa_utils.is_exit_robot(custom, e['robot_name']):
		continue

	is_add = False
	if rw_type == 'r':
		is_add = (e['sub'] == True)
	else:
		is_add = (e['sub'] == False)

	if is_add:
		entry = OrderedDict()
		entry['name'] = hakoniwa_utils.get_pdu_name(e)
		entry['class_name'] = hakoniwa_utils.get_pdu_class_name(rw_type)
		entry['conv_class_name'] = hakoniwa_utils.get_pdu_conv_class_name(rw_type, pkg_name)
		entry['topic_message_name'] = hakoniwa_utils.get_pdu_topic_name(e)
		entry['pdu_config_name'] = hakoniwa_utils.get_pdu_type_name(e)
		container.append(entry)

if custom != None:
	for e in custom['robots']:
		entry = OrderedDict()
		if rw_type == 'r':
			for e_list_entry in e['udp_pdu_readers']:
				entry['name'] = hakoniwa_utils.get_custom_pdu_name(e_list_entry)
				entry['class_name'] = e_list_entry['class_name']
				entry['conv_class_name'] = e_list_entry['conv_class_name']
				if len(e_list_entry['class_path']) > 0:
					entry['path'] = e_list_entry['class_path']
				if (len(e_list_entry['conv_class_path']) > 0):
					entry['conv_path'] = e_list_entry['conv_class_path']
				entry['pdu_config_name'] = e_list_entry['type']
		else:
			for e_list_entry in e['udp_pdu_writers']:
				entry['name'] = hakoniwa_utils.get_custom_pdu_name(e_list_entry)
				entry['class_name'] = e_list_entry['class_name']
				entry['conv_class_name'] = e_list_entry['conv_class_name']
				if len(e_list_entry['class_path']) > 0:
					entry['path'] = e_list_entry['class_path']
				if (len(e_list_entry['conv_class_path']) > 0):
					entry['conv_path'] = e_list_entry['conv_class_path']
				entry['pdu_config_name'] = e_list_entry['type']
		container.append(entry)

with open(out_dir + '/' + out_filename, mode='wt') as out_file:
  json.dump(container, out_file, ensure_ascii=False, indent=2)

