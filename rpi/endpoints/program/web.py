import threading

from fastapi import FastAPI, Request, BackgroundTasks
import uvicorn

from endpoints.program import BaseProgramEndpoint
from parsers import parse_gcode

class WebProgramController(BaseProgramEndpoint):
    app = FastAPI()

    @classmethod
    def _run_web_server(cls):
        uvicorn.run(cls.app, host="0.0.0.0", port=8000, log_level="critical")

    @classmethod
    def start(cls):
        threading.Thread(target=cls._run_web_server, daemon=True).start()

@WebProgramController.app.post("/gcode")
async def new_gcode(request: Request, background_tasks: BackgroundTasks):
    body = (await request.body()).decode("utf-8")
    background_tasks.add_task(parse_gcode, body)
