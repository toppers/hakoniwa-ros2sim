#!/usr/bin/python
# -*- coding: utf-8 -*-
import json
import sys
import glob
import re
import string
from jinja2 import Template, Environment, FileSystemLoader

env = Environment(loader=FileSystemLoader('./', encoding='utf8'))

if len(sys.argv) != 5:
	print "ERROR: generate.py <name> <in_dir> <out_dir> <package_name>"
	sys.exit()

out_name=sys.argv[1]
tpl_name=sys.argv[1] + ".tpl"
in_dir=sys.argv[2]
out_dir=sys.argv[3]
pkg_name=sys.argv[4]

class RosMessageContainer:
    pass

class RosMessage:
    pass

def to_conv(name):
	tmp = name.replace("int", "Int").replace("uInt", "UInt").replace("float", "Float").replace("string", "String").replace("bool", "Bool")
	return re.sub(r'\[.*\]', "Array", tmp)

def is_primitive(name):
	if (name == 'int8'):
		return True
	elif (name == 'uint8'):
		return True
	elif (name == 'int16'):
		return True
	elif (name == 'uint16'):
		return True
	elif (name == 'int32'):
		return True
	elif (name == 'uint32'):
		return True
	elif (name == 'int64'):
		return True
	elif (name == 'uint64'):
		return True
	elif (name == 'float32'):
		return True
	elif (name == 'float64'):
		return True
	elif (name == 'string'):
		return True
	elif (name == 'bool'):
		return True
	else:
		return False

def tplcode_type(name):
	if (name == 'int8'):
		return 'SByte'
	elif (name == 'uint8'):
		return 'Byte'
	elif (name == 'int16'):
		return 'Int16'
	elif (name == 'uint16'):
		return 'UInt16'
	elif (name == 'int32'):
		return 'Int32'
	elif (name == 'uint32'):
		return 'UInt32'
	elif (name == 'int64'):
		return 'Int64'
	elif (name == 'uint64'):
		return 'UInt64'
	elif (name == 'float32'):
		return 'float'
	elif (name == 'float64'):
		return 'double'
	elif (name == 'string'):
		return 'string'
	elif (name == 'bool'):
		return 'bool'
	else:
		return False


def is_array(name):
	if (name.find('[') > 0):
		return True
	else:
		return False

def is_primitive_array(name):
	if (is_array(name) and is_primitive(get_array_type(name))):
		return True
	else:
		return False

def get_array_type(name):
	tmp = name.split('[')
	return tmp[0].strip()

def get_type(name):
	if (is_array(name)):
		return get_array_type(name)
	else:
		return name

def get_msg_type(name):
	if (name.find('/') > 0):
		tmp = name.split('/')[1]
	else:
		tmp = name
	if (tmp == 'time'):
		return 'Time'
	else:
		return tmp


container = RosMessageContainer()
container.to_conv = to_conv
container.get_type = get_type
container.get_array_type = get_array_type
container.is_array = is_array
container.is_primitive = is_primitive
container.is_primitive_array = is_primitive_array
container.pkg_name = pkg_name
container.get_msg_type = get_msg_type
container.tplcode_type = tplcode_type
container.msg_pkgs = []

for line in open(in_dir + "/msg_pkg.txt", 'r'):
	line = line.capitalize()
	tmp = re.sub('_(.)',lambda x:x.group(1).capitalize(),line)
	container.msg_pkgs.append(tmp.strip())

file = open('./settings/' + pkg_name + '/RosTopics.json')
container.ros_topics = json.load(file)

container.msgs = []

container.accessor_type_name = out_name

files = glob.glob(in_dir + "/*.json")
for file in files:
    ros_msg = RosMessage()
    tmp = re.split('[./]', file)
    msg_name = tmp[len(tmp) - 2]
    ros_msg.name = msg_name
    container.msgs.append(ros_msg)
    tmp_file = open(in_dir + '/' + msg_name +'.json')
    ros_msg.json_data = json.load(tmp_file)
    if (msg_name == container.accessor_type_name):
    	container.accessor_json_data = ros_msg.json_data

tpl = env.get_template(in_dir + '/' + tpl_name)
out = tpl.render({'container':container})

print out.encode('utf-8')

