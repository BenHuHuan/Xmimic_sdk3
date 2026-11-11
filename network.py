"""DDS endpoint selection shared by wired and wireless offboard control."""
import ipaddress
import json
import subprocess
from pathlib import Path
from xml.sax.saxutils import escape

def interfaces():
    devices=json.loads(subprocess.check_output(['ip','-j','-4','addr','show'],text=True))
    return [dict(name=d['ifname'],kind=('loopback' if d['ifname']=='lo' else
                 'wireless' if (Path('/sys/class/net')/d['ifname']/'wireless').exists() else 'network'),
                 addresses=[a['local'] for a in d.get('addr_info',[]) if a['family']=='inet']) for d in devices]

def interface_ipv4(name):
    for device in interfaces():
        if device['name']==name:
            if not device['addresses']:raise ValueError(f'Interface {name} has no IPv4 address')
            return device['addresses'][0]
    raise ValueError(f'Interface {name} does not exist; run ./xrobot3 interfaces')

def dds_profile(local_ip,peer_ip):
    local=str(ipaddress.IPv4Address(local_ip));peer=str(ipaddress.IPv4Address(peer_ip))
    return f'''<?xml version="1.0"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors><transport_descriptor>
    <transport_id>selected_interface</transport_id><type>UDPv4</type>
    <interfaceWhiteList><address>{escape(local)}</address></interfaceWhiteList>
  </transport_descriptor></transport_descriptors>
  <participant profile_name="xrobot3_endpoint" is_default_profile="true"><rtps>
    <userTransports><transport_id>selected_interface</transport_id></userTransports>
    <useBuiltinTransports>false</useBuiltinTransports>
    <builtin><initialPeersList><locator><udpv4><address>{escape(peer)}</address></udpv4></locator></initialPeersList></builtin>
  </rtps></participant>
</profiles>'''
