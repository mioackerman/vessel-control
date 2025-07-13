import krpc
import floater

try:
    conn = krpc.connect(
        name='edward',
        address='192.168.2.29',
        rpc_port=50000,
        stream_port=50001
    )
    vessel = conn.space_center.active_vessel
    print("Connected to: ", vessel.name)
except Exception as e:
    print('Fail to connect: ', e)

if vessel.name == 'floater':
    print('Run floater protocol')
    floater.run(conn, vessel)



