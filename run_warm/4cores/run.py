import subprocess
import multiprocessing

script_folder = "./"

# List of script names
script_names = [
    "apsp.sh",
    "bc.sh",
    "cc.sh",
    #    'comm.sh',
    "pagerank.sh",
    "tc.sh",
    "tsp.sh",
    "sssp.sh",
    "bfs.sh",
    # Add the remaining script names here
]

# Loop through each script and run it
def run_script(script_name):
    script_path = script_folder + "/" + script_name
    subprocess.run(["bash", script_path])


pool = multiprocessing.Pool()

pool.map(run_script, script_names)

pool.cloes()
pool.join()
