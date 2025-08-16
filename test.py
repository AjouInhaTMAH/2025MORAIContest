import roslibpy

client = roslibpy.Ros(host='192.168.12.10', port=9090)
client.run()

if client.is_connected:
    print("연결 성공!")
else:
    print("연결 실패!")

client.terminate()
