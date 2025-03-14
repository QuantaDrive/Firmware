from fastapi import FastAPI, Body

app = FastAPI()

@app.post("/gcode")
async def new_gcode(body: str = Body(...)):
    pass