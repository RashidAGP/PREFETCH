import subprocess
import multiprocessing

script_folder = "./"

# List of script names
script_names = [
    "make_pr.sh",
    "make_XSBench.sh",
    "make_gups.sh",
    "make_dfs.sh"
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
