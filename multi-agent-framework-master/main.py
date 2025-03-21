from class_def.swarm import BoidsSwarm

# '''动捕环境Swarm'''
# node_ls = {
#     'boids_1': ['BoidsNode', '192.168.1.205', '12345', '1'],
#     'boids_2': ['BoidsNode', '192.168.1.200', '12345', '0'],
#     'boids_3': ['BoidsNode', '192.168.1.207', '12345', '0'],
#     'boids_4': ['BoidsNode', '192.168.1.201', '12345', '0'],
#     # 'boids_5': ['BoidsNode', '192.168.1.201', '12345', '0'],
#     # 'boids_6': ['BoidsNode', '192.168.1.202', '12345', '0'],
# }
# swarm = BoidsSwarm(node_ls)


# '''动捕虚实结合环境Swarm'''
# node_ls = {
#     'boids_1': ['VBoidsNode', '192.168.1.202', '12345', '0'],
#     'boids_2': ['VBoidsNode', '192.168.1.201', '12345', '0'],
#     'boids_3': ['VBoidsNode', '192.168.1.200', '12345', '0'],
# }
node_ls = {}
n_node = 4
for i in range(n_node):
    node_ls[f'Vboids_{i + 1}'] = ['VBoidsNode', '', '', '0']
swarm = BoidsSwarm(node_ls, params_style="Virtual")

#roslaunch vrpn_client_ros sample.launch server:=192.168.1.100
