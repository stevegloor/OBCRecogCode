#!/usr/bin/python
#ODROID U3 scanner program for OBC
#geosearch.py edited by Stephen Gloor
#added in pool multiprocessing for the ODROID U3
#
#Original code by Andrew Tridgell from cuav geosearch.py
#Directory scannning by Tim Golden - simple file scan
#FTP Code from
# Copyright (c) 2011, Xabier (slok) Larrakoetxea
# Copyright (c) 2011, Iraide (Sharem) Diaz
#
# 3 clause/New BSD license: 
# opensource: http://www.opensource.org/licenses/BSD-3-Clause
# wikipedia: http://en.wikipedia.org/wiki/BSD_licenses
# 373 has indiviual log files in results
# 374 has new calculation for thumbnail position
# 375 has ability to change the directory for FTP and
#     machine readable log file and removed saved images
# 376 has the new code for the compactness filter and altitude is zero by default


import numpy, os, time, cv, sys, math
import multiprocessing
import pyexiv2

from multiprocessing import Process, Pool

from  ftplib import FTP
from cuav.lib import cuav_util
from cuav.image import scanner
from cuav.lib import cuav_mosaic, mav_position,cuav_region
from cuav.camera import cam_params

#assist the process to be really multiCPU
os.system("taskset -p 0xff %d" % os.getpid())

radius_of_earth = 6378100.0 # in meters
fudge_factor = 20

USER = 'perthuav'
PASS = 'Nandos1'
SERVER = '192.168.42.129'
PORT = 13900
BINARY_STORE = True # if False then line store (not valid for binary files (videos, music, photos...))

def print_line(result):
	print(result)

def connect_ftp():
	#Connect to the server
	try:
		ftp = FTP()
		ftp.connect(SERVER, PORT)
		ftp.login(USER, PASS)
		return ftp
	except Exception:
		ftp = False
		
def upload_file(ftp_connection, upload_file_path):

	#Open the file
	try:
		upload_file = open(upload_file_path, 'r')

		#get the name
		path_split = upload_file_path.split('/')
		final_file_name = path_split[len(path_split)-1]

		try:
			#transfer the file
			print('Uploading ' + final_file_name + '...')
			if opts.ftpdestination:
				ftp_connection.cwd(opts.ftpdestination)
			if BINARY_STORE:
				ftp_connection.storbinary('STOR '+ final_file_name, upload_file)
			else:
				#ftp_connetion.storlines('STOR ' + final_file_name, upload_file, print_line)
				ftp_connection.storlines('STOR '+ final_file_name, upload_file)

			print('Upload finished.')
		except Exception:
			print ("Connection Lost ... ")
			
	except IOError:
		print ("No such file or directory... passing to next file")

def file_list(directory, extensions):
	'''return file list for a directory'''
	flist = []
	for (root, dirs, files) in os.walk(directory):
		for f in files:
			extension = f.split('.')[-1]
			if extension.lower() in extensions:
				flist.append(f)
	return flist

def upload(files1, ftp_conn):
	for f in files1:
		f = os.path.join(opts.destination, f)
		upload_file(ftp_conn,f)

def gps_newpos(lat, lon, bearing, distance):
	'''extrapolate latitude/longitude given a heading and distance 
	thanks to http://www.movable-type.co.uk/scripts/latlong.html
	'''
	from math import sin, asin, cos, atan2, radians, degrees

	lat1 = radians(lat)
	lon1 = radians(lon)
	brng = radians(bearing)
	dr = distance/radius_of_earth

	lat2 = asin(sin(lat1)*cos(dr) + cos(lat1)*sin(dr)*cos(brng))
	lon2 = lon1 + atan2(sin(brng)*sin(dr)*cos(lat1), cos(dr)-sin(lat1)*sin(lat2))
	return (degrees(lat2), degrees(lon2))

def pixel_coordinates(x, y, im_width, im_height, pos, mpp, delay):
	
	delay_distance = delay
   
   	midx = im_width/2
	midy = im_height/2
	xofs = math.fabs(x-midx)
	yofs = math.fabs(y-midy)

	bearing = math.degrees(math.atan2(xofs, yofs))
	
	true_bearing = 0
	
	if (x < midx) and (y < midy):
		true_bearing = pos.yaw - bearing 
		
	elif (x < midx) and (y > midy):
		true_bearing = pos.yaw - (180 - bearing) 
		
	elif (x > midx) and (y > midy):
		true_bearing = pos.yaw + (180 - bearing)
		
	elif 	(x > midx) and (y < midy):
		true_bearing = pos.yaw + bearing
		
	if bearing > 360:
		bearing = bearing - 360
	distance = (math.sqrt(xofs**2 + yofs**2)) * mpp

	actual_pos = gps_newpos(pos.lat, pos.lon, pos.yaw , delay_distance)
	return gps_newpos(actual_pos[0], actual_pos[1], true_bearing, distance)


def process(file):
	#Process one file
	scan_count = 0
	region_count = 0
	thumbs=[]
	regions=[]
	scan_parms = {
	        'MinRegionArea' : opts.min_region_area,
	        'MaxRegionArea' : opts.max_region_area,
	        'MinRegionSize' : opts.min_region_size,
	        'MaxRegionSize' : opts.max_region_size,
	        'MaxRarityPct'  : opts.max_rarity_pct,
	        'RegionMerge'   : opts.region_merge
	}

	f = os.path.join(opts.directory, file)
	print "Processing .... ", f
	
	pos = mav_position.exif_position(f)
	pos.time += opts.time_offset
	if opts.filter_type == 'compactness':
		calculate_compactness = True
	else:
		calculate_compactness = False
	
	im_full = cuav_util.LoadImage(f)
	(w,h) = cuav_util.image_shape(im_full)
	C_params = cam_params.CameraParams(lens=opts.lens, sensorwidth=opts.sensorwidth)
	C_params.set_resolution(w, h)
	im_full = numpy.ascontiguousarray(cv.GetMat(im_full))
	count = 0
	total_time = 0
	t0=time.time()
	img_scan = im_full

	if pos is not None:
		(sw,sh) = cuav_util.image_shape(img_scan)
		mpp = cuav_util.meters_per_pixel(pos, C=C_params)
		if mpp is not None:
			scan_parms['MetersPerPixel'] = mpp * (w/float(sw))
		regions = scanner.scan(img_scan, scan_parms)
	else:
		regions = scanner.scan(img_scan)
	regions = cuav_region.RegionsConvert(regions, cuav_util.image_shape(img_scan), cuav_util.image_shape(im_full), calculate_compactness)
	count += 1
	t1=time.time()
	frame_time = pos.time
	regions = cuav_region.filter_regions(im_full, regions, frame_time=frame_time, min_score=opts.minscore, filter_type=opts.filter_type)
	scan_count += 1
	region_count += len(regions)

	total_time += (t1-t0)
	
	if len(regions) > 0:
		composite = cuav_mosaic.CompositeThumbnail(cv.GetImage(cv.fromarray(im_full)), regions, 200)
		thumbs = cuav_mosaic.ExtractThumbs(composite, len(regions))
#		destinationfile = os.path.join(opts.destination, os.path.basename(f))            
#		cv.SaveImage(destinationfile, composite)
		index = 1
		for t in thumbs:
			destinationthumb = os.path.basename(f).split(".")
			destinationthumb = destinationthumb[0] + "_" + str(index) + "." + destinationthumb[1]
			destinationfile1 = os.path.join(opts.destination, destinationthumb)
			cv.SaveImage(destinationfile1, t)
			index = index + 1
			
		NameFile = os.path.basename(f).split(".")
		log_file = log_path +"/" + NameFile[0] + ".txt"
		fo = open(log_file, "a")
		print('Image %s %s %u regions' % (os.path.basename(f), pos, region_count))
		log_entry = os.path.basename(f) + "," + str(pos.lat) + "," + str(pos.lon) + "," + str(pos.altitude) + "," + str(pos.yaw) + "," + str(region_count)
		fo.write(log_entry + "\r\n")
		img_view = img_scan
		(wview,hview) = cuav_util.image_shape(img_view)
#		mat = cv.fromarray(img_view)
		ThumbPos = " "
		index = 1
		for r in regions:
			midx = (r.x1 + r.x2)//2
			midy = (r.y1 + r.y2)//2
			ThisThumbPos = pixel_coordinates(midx,midy,wview, hview, pos, mpp, opts.delay)
			ThumbDetails = str(index) + "," + str(r.scan_score) + "," + str(ThisThumbPos[0]) + "," + str(ThisThumbPos[1]) + "\r\n"
			fo.write(ThumbDetails)
#			r.draw_rectangle(mat, (255,0,0))
			index = index + 1
		fo.close()
	
def StartProcess(ProcessFile): #Function to call the processing
	process(ProcessFile) 
		
def StartUpload(filesbefore1): #Function to call the uploading
	while True:
		after1 = []          
		after1 = dict ([(g, None) for g in os.listdir (path_to_watch1)])
		added1 = [g for g in after1 if not g in filesbefore1]
		time.sleep(2.0)
		if added1:
			ftp_conn1 = connect_ftp()
			if ftp_conn1:
				upload(added1, ftp_conn1)
				filesbefore1 = after1   
			else:
				print ("No Connection Retrying")
		time.sleep(2.0)		

def parse_args():
	'''parse command line arguments'''
	from optparse import OptionParser
	file_type='str'
	directory_type='str'

	parser = OptionParser("geosearch.py [options] <directory>", description='GeoSearch')

	parser.add_option("--directory", default="/home/perthuav/files", type=directory_type, help="directory containing image files")
	parser.add_option("--destination", default="/home/odroid/results", type=directory_type, help="directory containing results files")
	parser.add_option("--savedir", default="/home/perthuav/saved", type=directory_type, help="directory containing results files")
	parser.add_option("--minscore", default=750, type='int', help="minimum score")
	parser.add_option("--filter-type", type='choice', default='simple', choices=['simple', 'compactness'], help="object filter type")
	parser.add_option("--delay", type='float', default=1.0, help="Delay time between picture and GPS Co-ord")
	parser.add_option("--time-offset", type='int', default=0, help="offset between camera and mavlink log times (seconds)")
	parser.add_option("--altitude", default=0, type='float', help="altitude (0 for auto)")
	parser.add_option("--ftpupload", action='store_true', default=False, help="ftp Upload")
	parser.add_option("--ftpdestination", default="/thumbs", type=directory_type, help="directory for ftp destination")
	parser.add_option("--sensorwidth", default=6.7, type='float', help="sensor width")
	parser.add_option("--lens", default=4.1, type='float', help="lens focal length")
	parser.add_option("--camera-params", default=None, type=file_type, help="camera calibration json file from OpenCV")
	parser.add_option("--roll-stabilised", default=False, action='store_true', help="roll is stabilised")
	parser.add_option("--fullres", action='store_true', default=False, help="scan at full resolution")
	parser.add_option("--min-region-area", default=0.003, type='float', help="minimum region area (m^2)")
	parser.add_option("--max-region-area", default=2.0, type='float', help="maximum region area (m^2)")
	parser.add_option("--min-region-size", default=0.05, type='float', help="minimum region size (m)")
	parser.add_option("--max-region-size", default=2.0, type='float', help="maximum region size (m)")
	parser.add_option("--region-merge", default=0.5, type='float', help="region merge size (m)")
	parser.add_option("--max-rarity-pct", default=0.02, type='float', help="maximum percentage rarity (percent)")
	return parser.parse_args()

if __name__ == '__main__':
	(opts, args) = parse_args()

    # main program
	log_path = opts.destination
	path_to_watch = opts.directory
	path_to_watch1 = opts.destination
	if opts.ftpupload:
		#Find any files already in the results directory
		before1 = dict ([(g, None) for g in os.listdir(path_to_watch1)])
		ftp_conn1 = connect_ftp()
		if ftp_conn1:
			upload(before1, ftp_conn1)
		else:
			print ("Cannot Upload")
			before1 = []
		#start the watching upload process
		r = Process(target=StartUpload, args=(before1,))
		r.start()
		
	#Find and process the files already in the photos directory
	before = dict ([(f, None) for f in file_list(path_to_watch, ['jpg', 'pgm', 'png'])])
	pool_size = 4
	pool = multiprocessing.Pool(processes=pool_size,)
	pool.map(StartProcess, before)
	pool.close() # no more tasks
	pool.join()
	#Loop looking for new files and then add them to a multiprocessing
	#pool for processing
	print "Ready to process ....."
	while True:
		after = []
		after = dict ([(f, None) for f in file_list(path_to_watch, ['jpg', 'pgm', 'png'])])
		added = [f for f in after if not f in before]
		time.sleep(1.0) #Wait until the files are fully downloaded
		if added: 
			try:
				pool_size = 4
				pool = multiprocessing.Pool(processes=pool_size,)
				pool.map(StartProcess, added)
				pool.close() # no more tasks
				pool.join()
			except Exception:
				print("Image bad")
		before = after 
		time.sleep(3.0)
	r.join()	
#end of main        
