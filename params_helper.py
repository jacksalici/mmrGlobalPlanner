import yaml

strs = ["# generated params loader for ROS"]

def process_yaml(data, prefix=""):
    for key, value in data.items():
        if isinstance(value, dict):
            strs.append(f"params_dict{"".join('[\'' + p +'\']' if p!="" else "" for p in prefix.split('.'))}['{key}'] = {'{}'}")
            process_yaml(value, prefix + key + ".")
        else:
            param_name = f"{prefix}{key}"
            strs.append(f"self.declare_parameter('{param_name}')")
            strs.append(f"params_dict{"".join('[\'' + p +'\']' if p!="" else "" for p in prefix.split('.'))}['{key}'] = self.get_parameter('{param_name}').value")


with open('config/params.yaml') as f:

    data = yaml.safe_load(f)
    process_yaml(data['/global_planner']['ros__parameters'])
    print(strs)
    
    file_path = "/tmp/generated_ros_param_loader.py"
    with open(file_path, "w") as file:
        file.write("\n".join(strs))
        
    import subprocess
    subprocess.run(["gedit", file_path])
        