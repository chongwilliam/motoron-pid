import yaml

if __name__ == '__main__':
    with open('motor_1.yaml', 'r') as yaml_file:
        data = yaml.safe_load(yaml_file)

    print(data['kp_pos'])
