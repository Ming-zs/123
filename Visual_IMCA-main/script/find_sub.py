
import os  
import yaml

RM_PATH = "/home/experi/experi/t" # 环境变量

def readYAML(f_name):
    with open(f_name, "r", encoding="utf-8") as f:
        result = yaml.load(f.read(), Loader=yaml.FullLoader)
        return result

def file_yaml(file_dir):   
    L=[]   
    for dirpath, dirnames, filenames in os.walk(file_dir):  
        for file in filenames :  
            if os.path.splitext(file)[1] == '.yaml':  
                L.append(os.path.join(dirpath, file))  
    return L

mod_txt = open("/home/experi/experi/t/script/ModuleList.txt", "w")

yaml_list = file_yaml("/home/experi/experi/t/pkg")

i = 0

for file in yaml_list:
    right_index = str(file).rfind('/')
    print(file[len(RM_PATH) + 5:right_index])
    mod_txt.write(file[len(RM_PATH) + 5:right_index])
    if(i != len(yaml_list) - 1):
        mod_txt.write(" ")
        i = i + 1

mod_txt.close()
