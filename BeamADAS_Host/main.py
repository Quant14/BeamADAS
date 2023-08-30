#This script will gather sensor information from BeamNG.tech and send it to the external controller for processing
from beamngpy import BeamNGpy, Scenario, Vehicle

# Instantiate BeamNGpy instance running the simulator from the given path,
# communicating over localhost:4771
bng = BeamNGpy('localhost', 4771, home='..\\bngtechdir.txt', user='..\\bngtechuserdir.txt')

bng.open()

scenario = Scenario('west_coast_usa', 'test')

vehicle = Vehicle('ego_vehicle', model='etk800', license='TEST')

scenario.add_vehicle(vehicle, pos=(-717, 101, 118), rot_quat=(0, 0, 0.3826834, 0.9238795))

scenario.make(bng)

bng.scenario.load(scenario)
bng.scenario.start()

vehicle.ai.set_mode('span')
input('Hit enter when done')