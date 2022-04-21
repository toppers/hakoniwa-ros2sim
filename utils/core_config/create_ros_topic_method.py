#!/usr/bin/python
# -*- coding: utf-8 -*-
import json
import sys
import glob
import re
from collections import OrderedDict

if len(sys.argv) != 3:
	print "Usage: " + sys.argv[0] + " <pkg_name> <out_dir>"
	sys.exit()

pkg_name=sys.argv[1]
out_dir=sys.argv[2]

out_data = OrderedDict()

out_data['name'] = 'ros_topic_io'
out_data['class_name'] = 'Hakoniwa.PluggableAsset.Communication.Method.ROS.' + pkg_name + '.RosTopicIo'
out_data['parameters'] = './unity_ros_params.json'

with open(out_dir + '/ros_topic_method.json', mode='wt') as out_file:
  json.dump(out_data, out_file, ensure_ascii=False, indent=2)


