import rclpy
from std_srvs.srv import Trigger

rclpy.init()
n = rclpy.create_node('test')
c = n.create_client(Trigger, '/stand_up')
print('waiting for /stand_up...')
ok = c.wait_for_service(timeout_sec=5)
print(f'available: {ok}')
if ok:
    f = c.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(n, f, timeout_sec=10)
    if f.done():
        r = f.result()
        print(f'success={r.success} msg={r.message}')
    else:
        print('timeout')
else:
    print('service not available')
n.destroy_node()
rclpy.shutdown()
