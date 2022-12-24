import sys
import xpc

client = xpc.XPlaneConnect()

client.sendDREFs(["sim/flightmodel2/controls/wingsweep_ratio","sim/cockpit2/controls/wingsweep_ratio"],[0,0])
