
import yaml

def load_criteria(path, task_id):
    with open(path) as f:
        data = yaml.safe_load(f)
    data["task_id"] = task_id
    return data
