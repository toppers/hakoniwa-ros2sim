#!/usr/bin/python
# -*- coding: utf-8 -*-
import json
import sys
import glob
import re
from collections import OrderedDict

if len(sys.argv) != 4:
	print "Usage: " + sys.argv[0] + " <ros_tpics> <pkg_name> <out_dir>"
	sys.exit()

in_file=sys.argv[1]
pkg_name=sys.argv[2]
out_dir=sys.argv[3]

file = open(in_file)
ros_topics = json.load(file)
if len(ros_topics['fields']) == 0:
	sys.exit()

out_data = OrderedDict()

out_data['name'] = 'ros_topic_io'
out_data['class_name'] = 'Hakoniwa.PluggableAsset.Communication.Method.ROS.' + pkg_name + '.RosTopicIo'
out_data['parameters'] = './unity_ros_params.json'

with open(out_dir + '/ros_topic_method.json', mode='wt') as out_file:
  json.dump(out_data, out_file, ensure_ascii=False, indent=2)


