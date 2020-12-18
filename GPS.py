from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
from enum import Enum
from threading import Thread, Timer
from multiprocessing import Process
import time, argparse, math
import matplotlib.pyplot as plt
import matplotlib

############################################################################
# Settings
############################################################################

# Connect to vehicle with this string
connection_string = '192.168.56.1:14550'
DELAY = 0.2
MAV_MODE_AUTO = 4
WAYPOINTS_COUNT = 1

# Parse connection argument and add preset flight paths
parser = argparse.ArgumentParser()
parser.add_argument("-c", "--connect", help="connection string")
args = parser.parse_args()
if args.connect:
    connection_string = args.connect

'''print("""Please open QGroundControl and Gazebo if you have not already done so.
        You can open QGroundControl by navigating to the QGroundControl.AppImage file you downloaded
        and run ./QGroundControl.AppImage.
        You can open Gazebo by navigating to the Firmware repository you cloned and running
        make posix gazebo_plane.""")'''


###########################################################################
# Mission Manager Class
###########################################################################

# This class is the main class for creating missions and running them
class MissionManager:
    #  Class variables for command types
    class command_type(Enum):
        COMMAND_TAKEOFF = 0
        COMMAND_WAYPOINT = 1
        COMMAND_LAND = 2

    count = 0

    # Create vehicle
    vehicle = connect(ip=connection_string, wait_ready=True)
    home_position_set = False

    # Create commands
    cmds = vehicle.commands

    # set home location
    home = vehicle.location.global_relative_frame

    # Constructor will initialize the program and run it
    def __init__(self):
        # Connect to the vehicle
        print("Connecting")

        # Load commands
        self.cmds.clear()

    def PX4setMode(self, mavMode):
        self.vehicle._master.mav.command_long_send(self.vehicle._master.target_system,
                                                   self.vehicle._master.target_component,
                                                   mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavMode, 0, 0, 0, 0, 0, 0)

    def get_location_offset_meters(self, original_location, dNorth, dEast, alt):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
        specified `original_location`. The returned Location adds the entered `alt` value to the altitude of the `original_location`.
        The function is useful when you want to move the vehicle around specifying locations relative to
        the current vehicle position.
        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius = 6378137.0  # Radius of "spherical" earth

        # Coordinate offsets in radians
        dLat = dNorth / earth_radius
        dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

        # New position in decimal degrees
        newlat = original_location.lat + (dLat * 180 / math.pi)
        newlon = original_location.lon + (dLon * 180 / math.pi)
        return LocationGlobal(newlat, newlon, original_location.alt + alt)

    '''def print_vehicle_state(self):
        # Display basic vehicle state
        print(" Type: %s" % self.vehicle._vehicle_type)
        print(" Armed: %s" % self.vehicle.armed)
        print(" System status: %s" % self.vehicle.system_status.state)
        print(" GPS: %s" % self.vehicle.gps_0)
        print(" Alt: %s" % self.current_location.alt)'''

    def create_command(self, cmd_type, wp, vertical, horizontal, depth):
        if (cmd_type == self.command_type.COMMAND_TAKEOFF):
            print("In takeoff \n")
            # wp = self.get_location_offset_meters(self.home, vertical, horizontal, depth)
            wp = LocationGlobal(vertical, horizontal, depth)

            cmd = Command(0, 0, 0,
                          mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                          mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                          0, 1, 0, 0, 0, 0,
                          wp.lat, wp.lon, wp.alt)

        elif (cmd_type == self.command_type.COMMAND_WAYPOINT):
            if vertical != self.home.lat and horizontal != self.home.lon:
                wp = self.get_location_offset_meters(self.home, vertical, horizontal, depth)
            else:
                wp = LocationGlobal(vertical, horizontal, depth)

            cmd = Command(0, 0, 0,
                          mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                          mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                          0, 1, 0, 0, 0, 0,
                          wp.lat, wp.lon, wp.alt)

        elif (cmd_type == self.command_type.COMMAND_LAND):
            if vertical != self.home.lat and horizontal != self.home.lon:
                wp = self.get_location_offset_meters(self.home, vertical, horizontal, depth)
            else:
                wp = LocationGlobal(vertical, horizontal, depth)

            cmd = Command(0, 0, 0,
                          mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                          mavutil.mavlink.MAV_CMD_NAV_LAND,
                          0, 1, 0, 0, 0, 0,
                          wp.lat, wp.lon, wp.alt)
        else:
            print("Invalid command type")
            return

        self.cmds.add(cmd)
        return wp

    def run_mission(self):
        # wait for a home position lock
        while not self.home_position_set:
            print("Waiting for home position...")
            time.sleep(1)

        # Print vehicle states
        # self.print_vehicle_state()

        # Change to AUTO mode
        self.PX4setMode(MAV_MODE_AUTO)
        time.sleep(1)

        # Upload waypoints
        wp = ""
        wp = self.create_command(self.command_type.COMMAND_TAKEOFF, wp, self.home.lat, self.home.lon, 5)

        waypoints_count = read_numbers("How many waypoints do you want to set?")

        for i in range(0, waypoints_count):
            add_waypoint(wp, i)

        self.create_command(self.command_type.COMMAND_LAND, wp, self.home.lat, self.home.lon, 5)

        # Upload mission
        self.cmds.upload()
        time.sleep(2)

        # Arm vehicle
        self.vehicle.armed = True

        Thread(target=self.gps_status).start()

        # Monitor mission execution
        nextwaypoint = self.vehicle.commands.next
        while (nextwaypoint < len(self.vehicle.commands)):
            if self.vehicle.commands.next > nextwaypoint:
                # display_seq = self.vehicle.commands.next + 1
                print("Moving to waypoint %s" % self.vehicle.commands.next)
                nextwaypoint = self.vehicle.commands.next
            time.sleep(1)

        # Wait for the vehicle to land
        while (self.vehicle.commands.next > 0):
            time.sleep(1)

        # Disarm vehicle
        self.vehicle.armed = False
        time.sleep(1)

        # Close vehicle object before exiting script
        self.vehicle.close()
        time.sleep(1)

        print("Vehicle set")

    lats = []
    lons = []

    Estimated_lats = []
    Estimated_lons = []

    def gps_status(self):
        current_location = self.vehicle.location.global_relative_frame
        Timer(DELAY, self.gps_status).start()
        self.count += 1

        print("Current lat is: %s" % current_location.lat)
        print("Current lon is: %s \n" % current_location.lon)

        if self.count > 6:
            accuracy(Estimated.lat, Estimated.lon, current_location.lat, current_location.lon)

        self.lats.append(current_location.lat)
        self.lons.append(current_location.lon)

        if self.count > 5:
            moths(self.lats, self.lons)
            print("Estimated latitude: %s" % Estimated.lat)
            print("Estimated longitude: %s\n" % Estimated.lon)

            self.Estimated_lats.append(Estimated.lat)
            self.Estimated_lons.append(Estimated.lon)


class Estimated:
    lat = 0
    lon = 0


class Accuracy:
    def __init__(self, distance, string):
        self.distance = distance
        self.string = string


class Plot:
    abort = False

    def draw(self):
        plt.ion()
        fig = plt.figure()
        axes = fig.add_subplot(111)

        axes.set_autoscale_on(True)
        axes.autoscale_view(True, True, True)

        self.true, = plt.plot([], [], "b")
        self.estimated, = plt.plot([], [], "r")
        plt.xlabel("True gps coordinates")
        plt.ylabel("Estimated gps coordinates")

        this_manager = plt.get_current_fig_manager()
        this_manager.window.setGeometry(5, 120, 800, 520)

        plt.show(block=False)

        while not Plot.abort:
            x = new_mission.lats
            y = new_mission.lons
            e_x = new_mission.Estimated_lats
            e_y = new_mission.Estimated_lons

            self.true.set_data(x, y)
            self.estimated.set_data(e_x, e_y)

            axes.relim()
            axes.autoscale_view(True, True, True)

            fig.canvas.draw()
            fig.canvas.flush_events()

    def key_pressed(self):
        Plot.abort = True


def moths(x, y):
    x_koefs = []
    y_koefs = []

    lat_error = 0
    lon_error = 0

    current_lat = x[len(x) - 1]
    current_lon = y[len(y) - 1]

    velocity_x = 1 / DELAY * abs(current_lat - x[len(x) - 2])
    velocity_y = 1 / DELAY * abs(current_lon - y[len(y) - 2])

    if Estimated.lat > 0 and (x[len(x) - 1] != x[len(x) - 2]):
        lat_error = current_lat - Estimated.lat
        lon_error = current_lon - Estimated.lon

    for i in range(0, len(x) - 1):
        x_koefs.append(x[i + 1] / x[i])
    for i in range(0, len(y) - 1):
        y_koefs.append(y[i + 1] / y[i])

    x_avg_k = sum(x_koefs) / len(x_koefs)
    y_avg_k = sum(y_koefs) / len(y_koefs)

    x_omega = x_koefs[len(x_koefs) - 1] - x_avg_k
    y_omega = y_koefs[len(y_koefs) - 1] - y_avg_k

    print("---------- Average Coefficients ---------")
    print("----------- x -------------- y ----------")
    print(x_avg_k, y_avg_k)
    print("")
    print("---------- Last element sub avg = omega ---------")
    print("-------------- x --------------- y --------------")
    print(x_omega, y_omega)
    print("")
    print("---------- Error ---------")
    print("---- lat -------- lon ----")
    print(lat_error, lon_error)
    print("")

    Estimated.lat = round(-DELAY * velocity_x * x_omega + current_lat * x_avg_k + lat_error, 7)
    Estimated.lon = round(-DELAY * velocity_y * y_omega + current_lon * y_avg_k + lon_error, 7)


def accuracy(e_lat, e_lon, t_lat, t_lon):
    lat_difference = abs(e_lat - t_lat)
    lon_difference = abs(e_lon - t_lon)

    a = accuracy_maths(lat_difference)
    b = accuracy_maths(lon_difference)

    print("The deviation is:")
    print("%s " % round(a.distance, 2) + a.string + " latitude")
    print("%s " % round(b.distance, 2) + b.string + " longitude\n")


def accuracy_maths(a):
    c = a * 10000000

    if c // 10000000 > 0:
        d = round(c // 100000, 2)
        string = "km"
    elif c // 1000000 > 0:
        d = round(c / 100000, 2)
        string = "km"
    elif c // 100000 > 0:
        d = round(c / 100000, 2)
        string = "km"
    elif c // 10000 > 0:
        d = round(c / 100, 2)
        string = "m"
    elif c // 1000 > 0:
        d = c / 100
        string = "m"
    elif c // 100 > 0:
        d = c / 100
        string = "m"
    elif c // 10 > 0:
        d = c
        string = "cm"
    else:
        d = c % 10
        string = "cm"

    return Accuracy(d, string)


def read_numbers(prompt=""):
    while True:
        val = input(prompt)
        try:
            val_int = int(val)
            return val_int
        except ValueError:
            if val != "home":
                print("The value is not a number\n")
            else:
                return val


def add_waypoint(wp, wp_number):
    print("Waypoint %s: " % (wp_number + 1))
    lat = read_numbers("Enter meters to go forwards (-backwards): ")
    lon = read_numbers("Enter meters to go right (-left): ")
    alt = read_numbers("Enter altitude (m): ")

    if lat == "home" or lon == "home" or alt == "home":
        lat = new_mission.home.lat
        lon = new_mission.home.lon
        alt = new_mission.home.alt

    new_mission.create_command(new_mission.command_type.COMMAND_WAYPOINT, wp, lat, lon, alt)
    print("")


# Listeners
@MissionManager.vehicle.on_message('HOME_POSITION')
def listener(self, name, home_position):
    global home_position_set
    MissionManager.home_position_set = True


# Run mission
new_mission = MissionManager()

matplotlib.use("Qt5Agg")

gps_graph = Plot()

Thread(target=new_mission.run_mission).start()

p1 = Process(target=gps_graph.draw())
p1.start()
