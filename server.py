# server.py
import asyncio
import tempfile
import subprocess
from aiohttp import web, WSMsgType
from controller import (
    drive_tank, control_camera_tilt, control_gripper_tilt,
    control_gripper, play_audio
)

routes = web.RouteTableDef()

@routes.get('/')
async def index(request):
    return web.FileResponse('./index.html')

@routes.get('/video')
async def video_stream(request):
    # Replace with actual video stream logic if needed
    return web.FileResponse('./static/sample.jpg')

@routes.get('/ws')
async def websocket_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    print("WebSocket connected")

    async for msg in ws:
        if msg.type == WSMsgType.TEXT:
            data = msg.json()

            if data['type'] == 'joystick':
                x = data['x']
                y = -data['y']
                left = y + x
                right = y - x
                drive_tank(left, right)

            elif data['type'] == 'cameraTilt':
                control_camera_tilt(data['value'])

            elif data['type'] == 'gripperTilt':
                control_gripper_tilt(data['value'])

            elif data['type'] == 'close':
                control_gripper("close")

            elif data['type'] == 'open':
                control_gripper("open")

            elif data['type'] == 'open_release':
                control_gripper("stop")

            elif data['type'] == 'close_release':
                control_gripper("stop")

            elif data['type'] == 'key':
                key = data['key'].upper()
                if key == 'W':
                    drive_tank(1.0, 1.0)
                elif key == 'S':
                    drive_tank(-1.0, -1.0)
                elif key == 'A':
                    drive_tank(-1.0, 1.0)
                elif key == 'D':
                    drive_tank(1.0, -1.0)
            
            elif data['type'] == 'key_release':
                key = data['key'].upper()
                if key in ['W', 'A', 'S', 'D']:
                    drive_tank(0.0, 0.0)

        elif msg.type == WSMsgType.ERROR:
            print("WebSocket error", ws.exception())
    return ws

@routes.get('/mic')
async def mic_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    print("Microphone WebSocket connected")

    buffer = b''
    async for msg in ws:
        if msg.type == WSMsgType.BINARY:
            buffer += msg.data
            if len(buffer) > 5000:
                pcm = await decode_audio(buffer)
                play_audio(pcm)
                buffer = b''
    return ws

async def decode_audio(webm_data):
    with tempfile.NamedTemporaryFile(suffix='.webm') as f_in, \
         tempfile.NamedTemporaryFile(suffix='.wav') as f_out:
        f_in.write(webm_data)
        f_in.flush()
        subprocess.run([
            'ffmpeg', '-i', f_in.name,
            '-ar', '16000', '-ac', '1', f_out.name
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return f_out.read()

app = web.Application()
app.add_routes(routes)

web.run_app(app, host='0.0.0.0', port=80)
