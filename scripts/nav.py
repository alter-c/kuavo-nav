import os
import argparse
import json
import time
import requests

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--command", type=str, required=True, help="command to execute")
    parser.add_argument("-id", "--cat_id", type=str, help="category id for navigation")
    parser.add_argument("-tid", "--task_id", type=str, help="task id for state query")
    args = parser.parse_args()

    if args.command == "start":
        # os.system(f"curl '0.0.0.0:8080/api/navigation/start?cid={args.cat_id}'")
        resp = requests.get(f"http://0.0.0.0:8080/api/navigation/start", params={"cid": args.cat_id})
        print(resp.text)
        print("##############################")
        
        res = json.loads(resp.text)
        task_id = res["data"]["task_id"]
        for i in range(30):
            state_resp = requests.get(f"http://0.0.0.0:8080/api/navigation/state", params={"task": task_id})
            state_json = json.loads(state_resp.text)
            task_state = state_json["data"]["status"]
            print(f"Task State: {task_state}")
            time.sleep(0.1)
    elif args.command == "stop":
        os.system(f"curl '0.0.0.0:8080/api/navigation/stop'")
    elif args.command == "state":
        os.system(f"curl '0.0.0.0:8080/api/navigation/state?task={args.task_id}'")
    else:
        print(f"Unknown command: {args.command}")

