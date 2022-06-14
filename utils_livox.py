import openpylivox as opl
import numpy as np
import open3d as o3d
import os
import time


def get_numpy_from_csv(filename):
    """
    Reads a CSV file and returns a numpy array
    """
    if not filename.endswith('.csv'):
        raise ValueError('Filename must end with .csv')
    data = np.genfromtxt("data/"+filename, delimiter=',')
    return np.expand_dims(data[:,:4],axis=0)


def record_sequence_xyzr(sensor=None, filename=str(time.strftime("%Y%m%d-%H%M%S")), duration=2):
    """
    Records a sequence of data from a sensor and saves it to a CSV file
    """
    if sensor is None:
        sensor = set_up_sensor() # Standard. Need to change IP adress for your computer and sensor
    if sensor is not None:
        print("Recording sequence... Smile!")
        sensor.lidarSpinUp()
        sensor.dataStart(save_to_csv=False)
        sensor.saveDataToNumpy("data/"+filename, secsToWait=0,duration=duration)
        print("Recording Completed.\nSave data to file: data/{}.csv".format(filename))
        nump = sensor.dataStop()
        sensor.lidarSpinDown()
        print("nump: {}".format(nump))
    return filename

def stream_live(sensor=None, filename=str(time.strftime("%Y%m%d-%H%M%S")), duration_stream=30,lidar_interval=0.1):
    """
    Records a sequence of data from a sensor and saves it to a CSV file
    """
    if sensor is None:
        sensor = set_up_sensor() # Standard. Need to change IP adress for your computer and sensor
    if sensor is not None:
        i = 0
        sensor.lidarSpinUp()
        print("Stream Start")
        start_time = 0
        vis = None
        first = True
        while time.time()-start_time < duration_stream or first:
            
            sensor.dataStart(save_to_csv=False,interval=lidar_interval)
            sensor.saveDataToNumpy("data/"+filename, secsToWait=0,duration=lidar_interval)
            xyzr = sensor.dataStop()
            
            i = i+1
            if xyzr is not None:
                print(f"xyzr shape: {xyzr.shape}. Frame: {i}")
                xyzr = np.expand_dims(xyzr,axis=0)
                if first:
                    vis, pcd = initialize_o3d_plot(xyzr)
                    start_time = time.time()
                    first = False
                update_open3d_live(pcd,xyzr,vis)
            else:
                print("Frame: {}".format(i))
        sensor.lidarSpinDown()
        print("Stream End")
    else:
        print("Failed to connect to sensor could not stream")
    return filename

def initialize_o3d_plot(xyzr):
    """
    Initialize o3d plot.
    @param xyzr: numpy array of xyzr data to initailize plot with.

    @return: o3d.visualization.Visualizer object
    """
    print("Initializing Open3D plot")
    xyz = xyzr[:,:,:3]
    signal = xyzr[:,:,3:]
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    geo = o3d.geometry.PointCloud()
    print("xyz.shape: {}".format(xyz.shape))
    geo.points = o3d.utility.Vector3dVector(xyz[0])
    vis.add_geometry(geo)
    vis.update_renderer()
    #vis.run()
    return vis,geo

def update_open3d_live(geo,xyzr,vis):
    """
    Plot lidar data live using open3d.
    @param xyzr: numpy array of xyzr data
    """
    xyz = xyzr[:,:,:3]
    signal = xyzr[:,:,3:]
    geo.points = o3d.utility.Vector3dVector(xyz[0])
    vis.update_geometry(geo)
    vis.poll_events()
    vis.update_renderer()
    
    
def plot_existing_open3d_pc(xyzr):
    """
    Plot lidar example from xyzr data.
    @param xyzr: numpy array of xyzr data. Each cloud is stacked in dim=0. Iterate over dim=0 to visualize all clouds.
    """
    xyz = xyzr[:,:,:3] 
    signal = xyzr[:,:,3:] # 3: for broadcasting
    print("Shape of xyz: {}".format(xyz.shape))
    print("Shape of signal: {}".format(signal.shape))
    #vis = o3d.visualization.Visualizer()
    #vis.create_window()
    #geo = o3d.geometry.PointCloud()
    #geo.points = o3d.utility.Vector3dVector(xyz[0]) # Initialize with first cloud
    vis, geo = initialize_o3d_plot(xyzr)

    for i,cloud in enumerate(xyz[1:],1):
        print("Frame: {}".format(i))
        geo.points = o3d.utility.Vector3dVector(cloud)
        vis.update_geometry(geo)
        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.2) # Time between clouds


def set_up_sensor(pc_ip="192.168.1.101",sens_ip="192.168.1.11", port_data=65000, port_command=55000):
    """
    Sets up a sensor and returns it
    """
    sensor = opl.openpylivox(True)
    sensor.showMessages = True
    try:
        connect = sensor.auto_connect()
    except:
        print("Failed to Auto Connect")
        print(f"Connecting maually to: {sens_ip} from {pc_ip}")
        connect = sensor.connect(pc_ip, sens_ip, port_data, port_command)
    return sensor if sensor._isConnected else None

if __name__ == '__main__':
    # get data from a sensor
    #filename = record_sequence_xyzr(duration=0.5)
    #filename = "20220614-130838"
    #pcd = get_numpy_from_csv(f'{filename}.csv')
    #plot_existing_open3d_pc(pcd)
    stream_live()
