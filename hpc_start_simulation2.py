import subprocess
import sys
import os.path

world_db = {
	'OneCluster':'$PWD/sources/w_swarm1/world_db/20180208_w_swarm1_circular_one_region_cluster.world',
	'TwoClusters':'$PWD/sources/w_swarm1/world_db/20180208_w_swarm1_circular_two_region_cluster.world',
	'FourClusters':'$PWD/sources/w_swarm1/world_db/20180209_w_swarm1_circular_four_clusters.world',
	'HalfCluster':'$PWD/sources/w_swarm1/world_db/20180209_w_swarm1_circular_half_cluster_half_uniform.world',
	'Uniform':'$PWD/sources/w_swarm1/world_db/20180209_w_swarm1_circular_uniform_litter.world',
	'OneCluster100m':'$PWD/sources/w_swarm1/world_db/20180208_w_swarm1_circular_one_region_cluster_100m.world',
	'TwoClusters100m':'$PWD/sources/w_swarm1/world_db/20180208_w_swarm1_circular_two_region_cluster_100m.world',
	'FourClusters100m':'$PWD/sources/w_swarm1/world_db/20180209_w_swarm1_circular_four_clusters_100m.world',
	'HalfCluster100m':'$PWD/sources/w_swarm1/world_db/20180209_w_swarm1_circular_half_cluster_half_uniform_100m.world',
	'Uniform100m':'$PWD/sources/w_swarm1/world_db/20180209_w_swarm1_circular_uniform_litter_100m.world',
	
}

def start_simulation(world_name,experiment, params_file, paramLine, sge_task_id, port_number, gzmode):
	port_number = int(port_number) + 11345
	folder_name = world_name + '-' + experiment

	print('folder_name: {}\nparamLine: {}\nport_number: {}\nsge_task_id: {}'.format(folder_name,paramLine,port_number,sge_task_id))
	copy_wp=None
	copy_rp=None
	copy_np=None
	copy_wg=None
	print('Checking control plugins')

	if os.path.isfile('compiled_plugins/libwp_swarm1.so'):
		copy_wp = 0
	else:
		copy_wp = 1

	if os.path.isfile('compiled_plugins/libmp_swarm1.so'):
		copy_rp = 0
	else:
		copy_rp = 1

	if os.path.isfile('compiled_plugins/libnest_plugin.so'):
		copy_np = 0
	else:
		copy_np = 1

	if os.path.isfile('./world_governor'):
		copy_wg = 0
	else:
		copy_wg = 1
	all_set = sum([copy_wp,copy_rp,copy_wg])

	#start up gazebo if all processes are successful
	if(all_set == 0):
		loadWorldStr = 'export GAZEBO_MASTER_URI=http://127.0.0.1:{};{} --verbose {}'.format(port_number,gzmode,world_db[world_name])
		load_world = subprocess.Popen(loadWorldStr,shell=True)#,stdin=subprocess.PIPE,stderr=subprocess.PIPE,stdout=subprocess.PIPEw_swarm1.world
		
		if load_world.returncode==None:
			load_logger = subprocess.Popen('export GAZEBO_MASTER_URI=http://127.0.0.1:{};./world_governor {} {} {} {}'.format(port_number, folder_name, params_file, paramLine, sge_task_id),shell=True)#,stdin=subprocess.PIPE,stderr=subprocess.PIPE,stdout=subprocess.PIPE,shell=True)
			if load_logger.returncode==None:
				print('''
				\n\n
				***********************************************
				All set: Simulation started and Logger loaded
				***********************************************
				\n\n''')

		while True:
			load_logger.poll()
			load_world.poll()
			#print(load_logger.returncode ,load_world.returncode)
			if load_logger.returncode != None or load_world.returncode != None:
				load_logger.kill()
				load_world.kill()
				print('''
				********************************
					Simulation Terminated
					World logger = {}
					World Status   = {}
				Process killed because of non "None"
				value.
				********************************
				'''.format(load_logger.returncode,load_world.returncode))
				#print('world logger status: ',load_logger.returncode,'world status: ',load_world.returncode)
				break
	else:
		print(all_set)

if __name__=='__main__':
	# paramLine and port_number vary based on $SGE_TASK_ID value
	# start_simulation(world_name, experiment, params_file, paramLine,   sge_task_id, port_number, gzmode)
	start_simulation(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6], sys.argv[7])