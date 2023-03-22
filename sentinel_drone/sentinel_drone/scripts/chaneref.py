import subprocess

from_SRS = "EPSG:4326"

to_SRS = "EPSG:4326"

src='task2d.tif'

dest= 'updated.tif'

cmd_list = ["gdalwarp","-r", "bilinear", "-s_srs", from_SRS, "-t_srs", to_SRS, "-overwrite", src, dest]

subprocess.run(cmd_list)