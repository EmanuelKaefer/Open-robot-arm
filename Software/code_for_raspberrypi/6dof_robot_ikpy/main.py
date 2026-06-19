from fastapi import FastAPI, WebSocket

app = FastAPI()

#send a test message
@app.get("/")
async def root():
    return {"message":"Server läuft"}

#open the Websocket server
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    while True:
        data = await websocket.receive_text()
        await websocket.send_text(f"You sent: {data}")