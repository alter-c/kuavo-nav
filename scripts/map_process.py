import json
import yaml


tags_to_remove = [
    "!!python/object:task_manager.TaskInfo",
    "!!python/object:__main__.RouteInfo",
    "!!python/object:__main__.Point2DInfo",
    "!!python/object:__main__.StationInfo"
]
def remove_tags(text, tags):
    for tag in tags:
        text = text.replace(tag, "")
    return text


# task process -------------------------------------------------
origin_task_file = '../configs/baai_2l/task.yaml'
with open(origin_task_file, 'r') as f:
    text = f.read()
text = remove_tags(text, tags_to_remove)
task_dict = yaml.safe_load(text)
id_to_pose = {
    task["id"]: task["pose"]
    for task in task_dict["Tasks"]
}

process_task_file = '../configs/task_map.json'
with open(process_task_file, 'r') as f:
    task_map = json.load(f)
for task in task_map.values():
    task['pose'] = id_to_pose[task['id']]
with open(process_task_file, 'w') as f:
    json.dump(task_map, f, indent=4)
# -------------------------------------------------------------


# route process ------------------------------------------------
origin_route_file = '../configs/baai_2l/route.yaml'
with open(origin_route_file, 'r') as f:
    text = f.read()
text = remove_tags(text, tags_to_remove)
route_dict = yaml.safe_load(text)
id_to_pose = {
    station["id"]:  (station["pose"][0], station["pose"][1])
    for station in route_dict["Stations"]
}

pose_to_id = {
    (station["pose"][0], station["pose"][1]): station["id"]
    for station in route_dict["Stations"]
}
route_json = {}
for i, route in enumerate(route_dict["Routes"]):
    ids = []
    for point in route["points"]:
        pose = tuple(point["point"])
        id = pose_to_id[pose]
        ids.append(str(id))
    route_json["route_{}".format(i)] = ids


process_route_file = '../configs/route_map.json'
with open(process_route_file, 'w') as f:
    file_data = {
        "points": id_to_pose,
        "routes": route_json
    }
    json.dump(file_data, f, indent=4)