using Hakoniwa.PluggableAsset.Communication.Pdu;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Hakoniwa.PluggableAsset.Communication.Pdu.Accessor
{
    public class {{container.accessor_type_name}}Accessor
    {
        private Pdu pdu;
{%- for item in container.accessor_json_data["fields"]: -%}
{%-		if (container.is_primitive(item["type"]) or container.is_primitive_array(item["type"])): %}
{%-		elif (container.is_array(item["type"])): %}
		private {{container.get_msg_type(container.get_array_type(item["type"]))}}Accessor[] pdu_{{container.get_array_type(item["name"])}}_array_accessor;
{%-		else: %}
		private {{container.get_msg_type(item["type"])}}Accessor pdu_{{item["name"]}}_accessor;
{%-		endif %}
{%- endfor %}
        
        public {{container.accessor_type_name}}Accessor(Pdu pdu)
        {
        	this.pdu = pdu;
{%- for item in container.accessor_json_data["fields"]: -%}
{%-		if (container.is_primitive(item["type"]) or container.is_primitive_array(item["type"])): %}
{%-		elif (container.is_array(item["type"])): %}
            var pdu_{{item["name"]}}_array = pdu.Refs("{{item["name"]}}");
            for (int i = 0; i < pdu_{{item["name"]}}_array.Length; i++)
            {
                this.pdu_{{container.get_array_type(item["name"])}}_array_accessor[i] = new {{container.get_msg_type(container.get_array_type(item["type"]))}}Accessor(pdu_{{item["name"]}}_array[i]);
            }
{%-		else: %}
			this.pdu_{{item["name"]}}_accessor = new {{container.get_msg_type(item["type"])}}Accessor(pdu.Ref("{{item["name"]}}"));
{%-		endif %}
{%- endfor %}
        }
        
{%- for item in container.accessor_json_data["fields"]: -%}
{%-	if (container.is_primitive(item["type"])): %}
        public {{container.tplcode_type(item["type"])}} {{item["name"]}}
        {
            set
            {
                pdu.SetData("{{item["name"]}}", value);
            }
            get
            {
                return pdu.GetData{{container.to_conv(item["type"])}}("{{item["name"]}}");
            }
        }
{%-	elif (container.is_primitive_array(item["type"])): %}
        public {{container.tplcode_type(container.get_array_type(item["type"]))}}[] {{item["name"]}}
        {
            get
            {
                return pdu.GetData{{container.to_conv(item["type"])}}("{{item["name"]}}");
            }
        }
{%-	elif (container.is_array(item["type"])): %}
        public {{container.get_msg_type(container.get_array_type(item["type"]))}}Accessor[] {{item["name"]}}
        {
            get
            {
                return pdu_{{item["name"]}}_array_accessor;
            }
        }
{%-	else: %}
        public {{container.get_msg_type(item["type"])}}Accessor {{item["name"]}}
        {
            set
            {
                pdu_{{item["name"]}}_accessor = value;
            }
            get
            {
                return pdu_{{item["name"]}}_accessor;
            }
        }
{%-	endif %}
{%- endfor %}
    }
}
