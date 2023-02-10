## simple test script to use x plane to determine the Y axis MOI of ask 21
import xpc
client = xpc.XPlaneConnect()

AA = client.getDREF("sim/aircraft/weight/acf_jyy_unitmass")

print(AA)
