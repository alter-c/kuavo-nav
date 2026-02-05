import os
import subprocess
import uvicorn
from fastapi import FastAPI
from pydantic import BaseModel


class PerformRequest(BaseModel):
    robot: str
    action: str
    mode: int # 夸父手臂控制模式

def check_robot_type(robot: str):
    robot_env = os.getenv("ROBOT_TYPE").lower()
    return robot == robot_env

app = FastAPI(title="Robot Action Service")
@app.get("/api/v1/status")
def get_status():
    return {
        "success": True,
        "code": 200,
        "message": "action service is running"
    }

@app.post("/api/v1/perform")
def perform_action(request: PerformRequest):
    robot = request.robot.lower()
    action = request.action.lower()
    mode = request.mode

    if not check_robot_type(robot):
        return {
            "success": False,
            "code": 500,
            "message": f"动作执行失败: Robot type error."
        }

    try:
        result = subprocess.run(
            ["./bin/action.sh", action, str(mode)],
            capture_output=True,
            timeout=10
        )

        if result.returncode == 0:
            return {
                "success": True,
                "code": 200,
                "message": "动作已经成功执行"
            }
        
    except Exception as e:
        return {
            "success": False,
            "code": 500,
            "message": f"动作执行失败: Unexpected error occurred: {str(e)}."
        }
    
@app.post("/api/v1/mode")
def change_mode(request: PerformRequest):
    robot = request.robot.lower()
    mode = request.mode

    if not check_robot_type(robot):
        return {
            "success": False,
            "code": 500,
            "message": f"模式切换失败: Robot type error."
        }

    try:
        cmd = ["./bin/mode.sh", str(mode)]
        result = subprocess.run(
            cmd,
            capture_output=True,
            timeout=10
        )

        if result.returncode == 0:
            return {
                "success": True,
                "code": 200,
                "message": "模式已经成功切换"
            }
        
    except Exception as e:
        return {
            "success": False,
            "code": 500,
            "message": f"模式切换失败: Unexpected error occurred: {str(e)}."
        }

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8091)