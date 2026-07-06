import sys
sys.path.insert(0, '/home/nvidia/gb_ws/sdk/genisom_l1_sdk-main/lib/zsl-1w/aarch64')
import mc_sdk_zsl_1w_py
import time

MD = {0: "PASSIVE", 1: "STAND", 10: "LIE_FREE", 18: "MOVE", 21: "ACTION", 51: "LIE_DOWN"}

app = mc_sdk_zsl_1w_py.HighLevel()
app.initRobot("192.168.168.216", 43988, "192.168.168.168")
time.sleep(2)

c = app.checkConnect()
b = app.getBatteryPower()
m = app.getCurrentCtrlmode()
r = app.getRPY()
p = app.getPosition()
v = app.getBodyVelocity()
g = app.getBodyGyro()

print("连接:", c)
print("电池:", b, "%")
print("模式:", m, "(%s)" % MD.get(m, "?"))
print("RPY: roll=%.3f pitch=%.3f yaw=%.3f" % (r[0], r[1], r[2]))
print("位置: x=%.3f y=%.3f z=%.3f" % (p[0], p[1], p[2]))
print("速度: vx=%.3f vy=%.3f vz=%.3f" % (v[0], v[1], v[2]))
print("角速度: gx=%.3f gy=%.3f gz=%.3f" % (g[0], g[1], g[2]))
